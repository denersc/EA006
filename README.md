# EA006 
Este repositório contém o código desenvolvido durante o trabalho final de curso "Estudo sobre sensores de flexão". 

O código está organizado em três diretórios. 

No diretório `arduino` está o software executado no Arduino durante a demonstração para leitura dos dois sensores.

No diretório `pd` está o patch Pure Data utilizado durante a demonstração

No diretório `controller`, temos o código do bloco de pré-processamento, com arquivos de configuração e calibragem de exemplo para execução do código. 

O código do bloco de pré-processamento depende das seguintes bibliotecas:

- [liblo](http://liblo.sourceforge.net/)
- [libserialport](https://sigrok.org/wiki/Libserialport)
- [cJSON](https://github.com/DaveGamble/cJSON)
- [glib](https://gitlab.gnome.org/GNOME/glib/)

Após instalação das dependências, basta executar `make` no diretório `controller` para compilar o bloco de pré-processamento. 

Após compilar, o binário pode ser executado da seguinte forma:

```
./controller -c <CONFIG_FILE> -a <CALIBRATION_FILE> [-t]
```

A flag `-t` é opcional e indica que o programa deve ser iniciado no modo de calibragem.
