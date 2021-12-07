#pragma once
#include <string>
#include "modbus.h"

using namespace std;

class TCP
{

public:
	TCP();
	TCP(string ip, uint16_t port, int slave_id);
	TCP(string ip, uint16_t port);
	TCP(string ip);


	void setupMobus(string ip, uint16_t port = MODBUS_TCP_DEFAULT_PORT, int slave_id = MODBUS_TCP_SLAVE);

	/**read function**/
	bool read_coils(int addr, int nb, uint8_t *dest);
	bool read_discrete_inputs(int addr, int nb, uint8_t *dest);
	bool read_holding_registers(int addr, int nb, uint16_t  *dest);
	bool read_input_registers(int addr, int nb, uint16_t *dest);
	/**write function**/
	bool write_coil(int addr, int status);
	bool write_coils(int addr, int nb, const uint8_t *src);
	bool write_holding_register(int addr, const uint16_t value);
	bool write_holding_registers(int addr, int nb, const uint16_t *src);
	bool write_and_read_holding_registers(int write_addr, int write_nb, const uint16_t *src, int read_addr, int read_nb, const uint16_t *dest);

	//    bool read_discrete_inputs(int addr, int nb, uint8_t *dest);
	//    bool read_holding_registers(int addr, int nb, uint16_t  *dest);
	//    bool read_input_registers(int addr, int nb, uint16_t *dest);

	void close();
	~TCP();
	bool connect();
	bool getConnected();
private:
	modbus_t* modbus_tcp;
	string host;
	uint16_t port;
	int slave_id;
	bool bConnected;
	string error_info(string cs, int errnum);
};

