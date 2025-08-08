

/*********************************************************
 * Functions
 *********************************************************/
void udp_socket_init(void);

esp_err_t udp_socket_send(char *data, int len);

int udp_socket_receive(char *buffer, int buffer_len);
