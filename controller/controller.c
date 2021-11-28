#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include <libserialport.h>
#include <lo/lo.h>
#include <cjson/cJSON.h>
#include <glib.h>

#define SYNC_BYTE 0xC7
#define PROGRAM_NAME "Controller"
#define MAX_CONFIG_SIZE 500000
#define MAX_CALIB_SIZE 50000
#define MAX_OUTPUTS 16

typedef enum {
    OUT_MAP_LINEAR,
    OUT_MAP_EXP,
    OUT_MAP_LOG,
    OUT_MAP_INVALID
} OutputMapping;

typedef enum {
    OUT_TYPE_CONTINUOUS,
    OUT_TYPE_DISCRETE,
    OUT_TYPE_THRESHOLD,
    OUT_TYPE_DIFFERENTIAL,
    OUT_TYPE_INVALID
} OutputType;

typedef struct _PArgs {
    gchar* cfg_file;
    gchar* calibration_file;
    gboolean calibrate;
} PArgs;

typedef struct _InCtx {
    gchar* label;
    uint16_t min;
    uint16_t max;
} InCtx;

typedef struct _OutCtx {
    size_t from_input;
    OutputMapping map;
    OutputType type;
    size_t opts_size;
    double* opts;
} OutCtx;

typedef struct _PCtx {
    // Calibration related
    FILE* calibration_file;

    // Input related
    gchar* in_device;
	struct sp_port* in_port;
	int in_bd;
	int in_n;
	InCtx* in_ctx;

    // Output related
    gchar* out_osc_addr;
    gchar *out_osc_port;
	char* out_osc_channel;
	lo_address out_osc;
	int out_n;
	OutCtx* out_ctx;

    // Processing related
    double *map_results;
    double *last_map_results;
    double *output;
} PCtx;
    

/* Logging */
void Log(const char* format, ...)
{
        va_list args;
        fprintf(stderr, "[%s] ", PROGRAM_NAME);
        va_start(args, format);
        vfprintf(stderr, format, args);
        va_end(args);
        fprintf(stderr, "\n");
}

void LogAndDie(const char* format, ...)
{
    va_list args;
    fprintf(stderr, "[%s] ", PROGRAM_NAME);
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
    fprintf(stderr, "\n");
    exit(1);
} 

/* Out Map/Type helpers */
OutputMapping output_mapping_from_string(gchar *s)
{
    if (s == NULL)
        return OUT_MAP_INVALID;
    if (!g_strcmp0(s, "linear"))
        return OUT_MAP_LINEAR;
    if (!g_strcmp0(s, "exp"))
        return OUT_MAP_EXP;
    if (!g_strcmp0(s, "log"))
        return OUT_MAP_LOG;
    return OUT_MAP_INVALID;
}

OutputType output_type_from_string(gchar *s)
{
    if (s == NULL)
        return OUT_TYPE_INVALID;
    if (!g_strcmp0(s, "continuous"))
        return OUT_TYPE_CONTINUOUS;
    if (!g_strcmp0(s, "discrete"))
        return OUT_TYPE_DISCRETE;
    if (!g_strcmp0(s, "threshold"))
        return OUT_TYPE_THRESHOLD;
    if (!g_strcmp0(s, "differential"))
        return OUT_TYPE_DIFFERENTIAL;
    return OUT_TYPE_INVALID;
}

gboolean output_type_check_n_opts(size_t n_opts, OutputType type)
{
    if (type == OUT_TYPE_CONTINUOUS && n_opts == 2)
        return TRUE;
    if (type == OUT_TYPE_DISCRETE && n_opts > 0)
        return TRUE;
    if (type == OUT_TYPE_THRESHOLD && n_opts == 1)
        return TRUE;
    if (type == OUT_TYPE_DIFFERENTIAL && n_opts == 1)
        return TRUE;
    return FALSE;
}

/* Helper function for error handling. */
int check(enum sp_return result)
{
	char* error_message;

	switch (result) {
	case SP_ERR_ARG:
		Log("Error: Invalid argument.\n");
		exit(1);
	case SP_ERR_FAIL:
		error_message = sp_last_error_message();
		Log("Error: Failed: %s\n", error_message);
		sp_free_error_message(error_message);
		exit(1);
	case SP_ERR_SUPP:
		Log("Error: Not supported.\n");
		exit(1);
	case SP_ERR_MEM:
		Log("Error: Couldn't allocate memory.\n");
		exit(1);
	case SP_OK:
	default:
		return result;
	}
}

float b2f(char* b)
{
    return (b[0] << 8) + b[1];
}


/* JSON Parsing */
void config_parse(PCtx* ctx, gchar* cfg_file)
{
    // Open cfg file, save to buf, close
    gchar buf[MAX_CONFIG_SIZE + 1];
    FILE* cfg = fopen(cfg_file, "r");
    if (cfg == NULL)
        LogAndDie("Erro: falha ao abrir arquivo de configuracao.");
    size_t cfg_size = fread(buf, sizeof(char), MAX_CONFIG_SIZE, cfg);
    if (ferror(cfg))
        LogAndDie("Erro: falha ao ler arquivo de configuracao");
    buf[cfg_size + 1] = '\0';
    fclose(cfg);

    // Parse JSON
    cJSON* cfg_json = cJSON_Parse(buf);
    if (cfg_json == NULL)
        LogAndDie("Erro: verifique se o arquivo de configuracao e um JSON valido.");

    // Get configs - INPUT
    cJSON* input = cJSON_GetObjectItemCaseSensitive(cfg_json, "input");
    if (!cJSON_IsObject(input))
        LogAndDie("Erro ao ler input na configuracao");

    cJSON* device = cJSON_GetObjectItemCaseSensitive(input, "device");
    if (!cJSON_IsString(device) || device->valuestring == NULL)
        LogAndDie("Erro ao ler input.device na configuracao");

    cJSON* baud_rate = cJSON_GetObjectItemCaseSensitive(input, "baud_rate");
    if (!cJSON_IsNumber(baud_rate))
        LogAndDie("Erro ao ler input.baud_rate na configuracao");

    cJSON* n_inputs = cJSON_GetObjectItemCaseSensitive(input, "n_inputs");
    if (!cJSON_IsNumber(n_inputs))
        LogAndDie("Erro ao ler input.n_inputs na configuracao");

    cJSON* labels = cJSON_GetObjectItemCaseSensitive(input, "labels");
    if (!cJSON_IsArray(labels))
        LogAndDie("Erro ao ler input.labels na configuracao");
    if (cJSON_GetArraySize(labels) != n_inputs->valueint)
        LogAndDie("Erro: na configuracao, quantidade de labels deve ser igual a input.n_inputs");

    // Save configs - INPUT
    ctx->in_device = g_strdup(device->valuestring);
    ctx->in_bd = baud_rate->valueint;
    ctx->in_n = n_inputs->valueint;
    ctx->in_ctx = malloc(ctx->in_n*sizeof(InCtx));
    int i = 0;
    cJSON* label = NULL;
    cJSON_ArrayForEach(label, labels)
    {
        if (!cJSON_IsString(label) || label->valuestring == NULL)
            LogAndDie("Erro ao ler input.labels[%d] na configuracao", i);
        ctx->in_ctx[i].label = g_strdup(label->valuestring);
        i++;
    }
    
    // Get configs - OUTPUT
    cJSON* output = cJSON_GetObjectItemCaseSensitive(cfg_json, "output");
    if (!cJSON_IsObject(output))
        LogAndDie("Erro ao ler output na configuracao");

    cJSON* osc_addr = cJSON_GetObjectItemCaseSensitive(output, "osc_addr");
    if (!cJSON_IsString(osc_addr) || osc_addr->valuestring == NULL)
        LogAndDie("Erro ao ler output.osc_addr na configuracao");

    cJSON* osc_port = cJSON_GetObjectItemCaseSensitive(output, "osc_port");
    if (!cJSON_IsString(osc_port) || osc_port->valuestring == NULL)
        LogAndDie("Erro ao ler output.osc_port na configuracao. Verifique se este  esta recebendo uma string");

    cJSON* osc_channel = cJSON_GetObjectItemCaseSensitive(output, "osc_channel");
    if (!cJSON_IsString(osc_channel) || osc_channel->valuestring == NULL)
        LogAndDie("Erro ao ler  output.osc_channel na configuracao");

    cJSON* n_outputs = cJSON_GetObjectItemCaseSensitive(output, "n_outputs");
    if (!cJSON_IsNumber(n_outputs))
        LogAndDie("Erro ao ler  output.n_outputs na configuracao");

    cJSON* params = cJSON_GetObjectItemCaseSensitive(output, "params");
    if (!cJSON_IsArray(params))
        LogAndDie("Erro ao ler output.params na configuracao");
    if (cJSON_GetArraySize(params) != n_outputs->valueint)
        LogAndDie("Erro: na configuracao, quantidade de params deve ser igual a output.n_outputs");

    // Save configs - OUTPUT
    ctx->out_osc_addr = g_strdup(osc_addr->valuestring);
    ctx->out_osc_port = g_strdup(osc_port->valuestring);
    ctx->out_osc_channel = g_strdup(osc_channel->valuestring);
    ctx->out_n = n_outputs->valueint;
    ctx->out_ctx = malloc(ctx->out_n*sizeof(OutCtx));
    ctx->map_results = malloc(ctx->out_n*sizeof(double));
    memset(ctx->map_results, 0, ctx->out_n*sizeof(double));
    ctx->last_map_results = malloc(ctx->out_n*sizeof(double));
    memset(ctx->last_map_results, 0, ctx->out_n*sizeof(double));
    ctx->output = malloc(ctx->out_n*sizeof(double));
    memset(ctx->output, 0, ctx->out_n*sizeof(double));
    i = 0;
    cJSON *param = NULL;
    cJSON_ArrayForEach(param, params)
    {
        if (!cJSON_IsObject(param))
            LogAndDie("Erro ao ler output.params[%d] na configuracao", i);

        cJSON* from_input = cJSON_GetObjectItemCaseSensitive(param, "from_input");
        if (!cJSON_IsNumber(from_input) || 
            from_input->valueint < 0 || 
            from_input->valueint > ctx->in_n - 1)
            LogAndDie("Erro ao ler output.params[%d].from_input na configuracao", i);

        cJSON* mapping = cJSON_GetObjectItemCaseSensitive(param, "mapping");
        if (!cJSON_IsString(mapping) ||
            mapping->valuestring == NULL)
            LogAndDie("Erro ao ler output.params[%d].mapping na configuracao", i);

        cJSON* type = cJSON_GetObjectItemCaseSensitive(param, "type");
        if (!cJSON_IsString(type) ||
            type->valuestring == NULL)
            LogAndDie("Erro ao ler output.params[%d].type na configuracao", i);

        ctx->out_ctx[i].from_input = from_input->valueint;
        ctx->out_ctx[i].map = output_mapping_from_string(mapping->valuestring);
        ctx->out_ctx[i].type = output_type_from_string(type->valuestring);
        if (ctx->out_ctx[i].map == OUT_MAP_INVALID ||
            ctx->out_ctx[i].type == OUT_TYPE_INVALID)
            LogAndDie("Erro: mapping ou type invalido em output.params[%d] na configuracao", i);

        cJSON* opts = cJSON_GetObjectItemCaseSensitive(param, "opts");
        if (!cJSON_IsArray(opts))
            LogAndDie("Erro ao ler output.params[%d].opts na configuracao", i);
        size_t opts_size = cJSON_GetArraySize(opts);
        Log("opts size is %d, type is %d", opts_size, ctx->out_ctx[i].type);
        if (!output_type_check_n_opts(opts_size, ctx->out_ctx[i].type))
            LogAndDie("Erro: quantidade de opts incorreta para output.params[%d].type escolhido", i);
        ctx->out_ctx[i].opts_size = opts_size;
        ctx->out_ctx[i].opts = malloc(opts_size*sizeof(double));

        cJSON *opt = NULL;
        int j = 0;
        cJSON_ArrayForEach(opt, opts)
        {
            if (!cJSON_IsNumber(opt))
                LogAndDie("Erro ao ler output.params[%d].opts[%d] na configuracao", i, j);
            ctx->out_ctx[i].opts[j] = opt->valuedouble;
            j++;
        }
        i++;
    }

    // Free and return
    cJSON_Delete(cfg_json);
}

void calib_parse(PCtx *ctx, gchar *calib_file)
{
    // Open calibration file, save to buf, close
    gchar buf[MAX_CALIB_SIZE + 1];
    FILE* calib = fopen(calib_file, "r");
    if (calib == NULL)
        LogAndDie("Erro: falha ao abrir arquivo de calibragem.");
    size_t calib_size = fread(buf, sizeof(char), MAX_CALIB_SIZE, calib);
    if (ferror(calib))
        LogAndDie("Erro: falha ao ler arquivo de calibragem");
    buf[calib_size + 1] = '\0';
    fclose(calib);

    // Parse JSON
    cJSON* calib_json = cJSON_Parse(buf);
    if (calib_json == NULL)
        LogAndDie("Erro: verifique se o arquivo de calibragem e um JSON valido.");

    // Parse and save values
    cJSON* data = cJSON_GetObjectItemCaseSensitive(calib_json, "data");
    if (!cJSON_IsArray(data))
        LogAndDie("Erro ao ler data na calibragem");
    size_t data_size = cJSON_GetArraySize(data);
    if (data_size < ctx->in_n)
        LogAndDie("Erro: arquivo de calibragem deve ter pelo menos tantos pares quanto entradas configuradas");
    int i = 0;
    cJSON *pair = NULL;
    cJSON_ArrayForEach(pair, data)
    {
        cJSON* min = cJSON_GetObjectItemCaseSensitive(pair, "min");
        if (!cJSON_IsNumber(min) || 
            min->valueint < 0)
            LogAndDie("Erro ao ler data[%d].min na calibragem.", i);
        cJSON* max = cJSON_GetObjectItemCaseSensitive(pair, "max");
        if (!cJSON_IsNumber(min) || 
            max->valueint < 0 ||
            max->valueint < min->valueint)
            LogAndDie("Erro ao ler data[%d].max na calibragem.", i);
        ctx->in_ctx[i].min = min->valueint;
        ctx->in_ctx[i].max = max->valueint;
        i++;
    }
    cJSON_Delete(calib_json);
}

/* Main and Calibrate Loop, processing functions */
double process_map(uint16_t input, uint16_t min, uint16_t max, OutputMapping map)
{
    static const double euler_constant = exp(1);
    if (input < min)
        return 0;
    if (input > max) 
        return 1;
    double in_d = (double)input;
    double min_d = (double)min;
    double max_d = (double)max;
    switch (map)
    {
        case OUT_MAP_LINEAR:
            return (in_d-min_d)/(max_d-min_d);
        case OUT_MAP_EXP:
            return (exp((in_d-min_d)/(max_d-min_d))-1)/(euler_constant-1);
        case OUT_MAP_LOG:
            return log(in_d/min_d)/log(max_d/min_d);
        default:
            break;
    }
    return 0;
}

double process_out(double input, OutputType type, size_t opts_size, double *opts, double last_input)
{
    switch (type)
    {
        case OUT_TYPE_CONTINUOUS:
            return (input*(opts[1] - opts[0])) + opts[0];
        case OUT_TYPE_DISCRETE:
            return opts[(size_t)(floor(input*(double)(opts_size)))];
        case OUT_TYPE_THRESHOLD:
            if (input >= opts[0])
                return 1;
            else 
                return 0;
        case OUT_TYPE_DIFFERENTIAL:
            if (fabs(last_input-input) > (1-opts[0]))
                return 1;
            else
                return 0;
        default:
            break;
    }
    return 0;
}

void process_and_send(PCtx *ctx, char *buf, size_t buf_size)
{
    // Process input
    for (int out = 0; out < ctx->out_n; out++)
    {
        // Save last results for differential output
        ctx->last_map_results[out] = ctx->map_results[out];

        size_t in = ctx->out_ctx[out].from_input;
        uint16_t input = (buf[in*2+1] << 8) + buf[in*2+2];
        ctx->map_results[out] = process_map(input,
                                            ctx->in_ctx[in].min,
                                            ctx->in_ctx[in].max,
                                            ctx->out_ctx[out].map);

        ctx->output[out] = process_out(ctx->map_results[out],
                                       ctx->out_ctx[out].type,
                                       ctx->out_ctx[out].opts_size,
                                       ctx->out_ctx[out].opts,
                                       ctx->last_map_results[out]);
    }
    lo_send(ctx->out_osc,
            ctx->out_osc_channel,
            "ffffffff",
            (float) ctx->output[0],
            (float) ctx->output[1],
            (float) ctx->output[2],
            (float) ctx->output[3],
            (float) ctx->output[4],
            (float) ctx->output[5],
            (float) ctx->output[6],
            (float) ctx->output[7]);
}

int main_loop(PCtx *ctx)
{
    size_t readSize = 2*(ctx->in_n) + 1;
    char *buf = malloc(readSize*sizeof(char));
    unsigned int timeout = 1000;
    int result;

    while (1)
    {
		result = check(sp_blocking_read(ctx->in_port, buf, readSize, timeout));
        if (result == readSize)
        {
            if ((uint8_t)buf[0] != SYNC_BYTE)
                Log("Aviso: perda de sincronia.");
            else
                process_and_send(ctx, buf, readSize);
        }
		else
			Log("Timed out, %d/%d bytes received.\n", result, readSize);
	}
}

void calibration_save_to_file(PCtx *ctx)
{
    cJSON *calib = cJSON_CreateObject();
    cJSON *data = cJSON_CreateArray();
    cJSON_AddItemToObject(calib, "data", data);
    for (int i = 0; i < ctx->in_n; i++)
    {
        cJSON *entry = cJSON_CreateObject();
        cJSON_AddItemToArray(data, entry);
        cJSON *min = cJSON_CreateNumber(ctx->in_ctx[i].min);
        cJSON *max = cJSON_CreateNumber(ctx->in_ctx[i].max);
        cJSON_AddItemToObject(entry, "min", min);
        cJSON_AddItemToObject(entry, "max", max);
    }
    char *json_string = cJSON_Print(calib);
    fputs(json_string, ctx->calibration_file);
    fclose(ctx->calibration_file);

}

gboolean calibration_get_mean(uint16_t *samples, uint16_t *result)
{
    uint64_t mean = 0;
    uint32_t max = 0;
    uint32_t min = 0xFFFFFFFF;
    for (int i = 0; i < 256; i++)
    {
        if (samples[i] < min)
            min = samples[i];
        if (samples[i] > max)
            max = samples[i];
        mean += samples[i];
    }
    mean = mean/256;

    Log("mean %d max %d min %d", mean, max, min);
    if ((double)mean * 1.10 < max ||
        (double)mean * 0.90 > min)
        return FALSE;

    *result = (uint16_t) mean;
    return TRUE;
}

int calibration_loop(PCtx *ctx)
{
    size_t readSize = 2*(ctx->in_n) + 1;
    char *buf = malloc(readSize*sizeof(char));
    unsigned int timeout = 1000;
    int result;
    uint16_t samples[256];

    // For each sensor
    Log("Calibragem iniciando");
    int i = 0, j = 0, k = 0;
    int error_count;
    while(i < ctx->in_n)
    {
        //log("Calibrando Sensor %s [%d]", ctx->in_ctx[i]->label, i);
        j = 0;
        while (j < 2)
        {
            if (j == 0)
                Log("Calibrando minimo do sensor %s. Coloque o sensor na maior flexao a ser utilizada", ctx->in_ctx[i].label);
            else
                Log("Calibrando maximo do sensor %s. Coloque o sensor na menor flexao a ser utilizada", ctx->in_ctx[i].label); 
            Log("Medidas comecam em 3 segundos");
            sleep(3);

            Log("Tirando medidas!");
            k = 0;
            error_count = 0;
            while (k < 256)
            {
                result = check(sp_blocking_read(ctx->in_port, buf, readSize, timeout));
                if (result == readSize)
                {
                    if ((uint8_t)buf[0] != SYNC_BYTE)
                        error_count++;
                    else
                    {
                        uint16_t input = (buf[i*2+1] << 8) + buf[i*2+2];
                        samples[k] = input;
                        k++;
                    }
                }
                else
                    error_count++;
                if (k%100 == 0 && k != 0)
                    Log("%d/256 medidas tiradas", k);
                if (error_count > 10)
                    LogAndDie("Erro: falha na comunicacao serial. Verifique a conexao");
            }
            uint16_t result;
            gboolean valid = calibration_get_mean(samples, &result);
            if (!valid)
                Log("Resultado inconsistente. Recomecando tentativa");
            else
            {
                Log("Sucesso! Iniciando proxima medida.");
                if (j == 0)
                    ctx->in_ctx[i].max = result;
                else
                    ctx->in_ctx[i].min = result;
                j++;
            }
        }
        i++;
    }

    Log("Calibragem concluida. Salvando no arquivo de calibragem...");
    calibration_save_to_file(ctx);
    Log("Sucesso!");
    return 0;
}

int main(int argc, char** argv)
{
    // Parse/check arguments
    PArgs* args = malloc(sizeof(PArgs));
    args->cfg_file = NULL;
    args->calibration_file = NULL;
    args->calibrate = FALSE;
    GOptionContext* opt_ctx = NULL;
    GOptionGroup* opt_grp = NULL;
    GError* g_err = NULL;
    GOptionEntry entries[] =
	{
		{"config-arquivo", 'c', G_OPTION_FLAG_NONE, G_OPTION_ARG_FILENAME, &(args->cfg_file),
			"Arquivo de configuracao", "CONFIG_ARQUIVO"},
		{"calibra-arquivo", 'a', G_OPTION_FLAG_NONE, G_OPTION_ARG_FILENAME, &(args->calibration_file),
			"Arquivo de calibragem. Executando no modo de calibragem, resultados serao escritos neste arquivo. Caso contrario, sera lido.", "CALIBRA_ARQUIVO"},
		{"calibra", 't', G_OPTION_FLAG_NONE, G_OPTION_ARG_NONE, &(args->calibrate),
			"Executar no modo de calibragem", NULL},
		{NULL}
	};
    opt_ctx = g_option_context_new("Controlador Serial para OSC");
    opt_grp = g_option_group_new("Opcoes", "", "", NULL, NULL);
    g_option_group_add_entries(opt_grp, entries);
    g_option_context_set_main_group(opt_ctx, opt_grp);
    if (!g_option_context_parse(opt_ctx, &argc, &argv, &g_err))
        LogAndDie("Erro %s\n", g_err->message);
    gchar* ctx_help = g_option_context_get_help(opt_ctx, TRUE, NULL);
    if (args->cfg_file == NULL)
    {
        Log("Erro: arquivo de configuracao nao especificado");
        Log("%s", ctx_help);
        exit(1);
    }
    if (args->calibration_file == NULL)
    {
        Log("Erro: arquivo de calibragem nao especificado");
        Log("%s", ctx_help);
        exit(1);
    }

    // Create new program context, populate it with config file info 
    PCtx* ctx = malloc(sizeof(PCtx));
    Log("Lendo arquivo de configuracao...");
    config_parse(ctx, args->cfg_file);
    Log("Sucesso!");

    // Check if running calibration mode
    if (args->calibrate)
    {
        Log("Abrindo arquivo de calibragem para escrita...");
        ctx->calibration_file = fopen(args->calibration_file, "w");
        if (ctx->calibration_file == NULL)
            LogAndDie("Erro: falha ao abrir arquivo de calibragem.");
        Log("Sucesso!");
    }
    
    else
    {
        Log("Lendo arquivo de calibragem...");
        calib_parse(ctx, args->calibration_file);
        Log("Sucesso!");
    }

    // Open and configure serial port
    Log("Procurando porta serial %s.", ctx->in_device);
    check(sp_get_port_by_name(ctx->in_device, &(ctx->in_port)));
    Log("Abrindo porta:");
    check(sp_open(ctx->in_port, SP_MODE_READ));
    Log("Configurando porta com %d 8N1, sem controle de fluxo.", ctx->in_bd);
    check(sp_set_baudrate(ctx->in_port, ctx->in_bd));
    check(sp_set_bits(ctx->in_port, 8));
    check(sp_set_parity(ctx->in_port, SP_PARITY_NONE));
    check(sp_set_stopbits(ctx->in_port, 1));
    check(sp_set_flowcontrol(ctx->in_port, SP_FLOWCONTROL_NONE));
    Log("Sucesso!");


    // Config OSC address if not running calibration mode
    if (!args->calibrate)
        ctx->out_osc  = lo_address_new_with_proto(LO_UDP, 
                                                  ctx->out_osc_addr, 
                                                  ctx->out_osc_port);

    if (!args->calibrate)
        return main_loop(ctx);
    else
        return calibration_loop(ctx);

    return 0;
}


