BUGS:

- Cuando el esp32 no esta conectado la orin guarda los mensajes hasta que el esp32 se conecte,
cuando se conecta los envia todos, pero cuando vuelves a desconectar el esp32 da segmentation fault o se siguen enviando mensajes
en vez de almacenarlos

- La esp32 no esta recibiendo bien los mensajes, ademas ahora esta mandando un monton de basura