#include "TCP.h"
#include "appexception.h"
TCP::TCP()
{

}


TCP::TCP(string ip, uint16_t port, int slave_id)
{
	setupMobus(ip, port, slave_id);
}
TCP::TCP(string ip, uint16_t port) {
	setupMobus(ip, port);
}

TCP::TCP(string ip) {
	setupMobus(ip);
}

void TCP::setupMobus(string ip, uint16_t port, int slave_id) {
	host = ip;
	this->port = port;
	bConnected = false;
	modbus_tcp = modbus_new_tcp(host.c_str(), port);
	modbus_set_error_recovery(modbus_tcp, MODBUS_ERROR_RECOVERY_LINK_AND_PROTOCOL);
	modbus_set_byte_timeout(modbus_tcp, 0, 0);
	if (!modbus_tcp) {
		throw PLCException("Unable to allocate modbus context");
		//    fprintf(stderr, "Unable to allocate modbus context\n");
		return;
	}
	int rc = modbus_set_slave(modbus_tcp, slave_id);
	if (rc == -1) {
		throw PLCException("Invalid slave ID");
		//      fprintf(stderr, "Invalid slave ID\n");
		modbus_free(modbus_tcp);
	}
}

bool TCP::connect() {
	int res;
	if (!bConnected) {
		res = modbus_connect(modbus_tcp);
		if (res == -1) {
			modbus_free(modbus_tcp);
			modbus_tcp = 0;
			bConnected = false;
			//
			throw PLCCommException(error_info("Connection failed: ", errno));
		}
		else bConnected = true;
	}
	return bConnected;
}

void TCP::close() {
	if (modbus_tcp && bConnected) {
		modbus_close(modbus_tcp);
	}
	bConnected = false;
}
string TCP::error_info(string cs, int errnum) {
	string s = cs + modbus_strerror(errno);
	return s;
}
/*The result of reading is stored in dest array as unsigned bytes (8 bits) set to TRUE or FALSE*/
bool TCP::read_coils(int addr, int nb, uint8_t *dest) {
	int res = modbus_read_bits(modbus_tcp, addr, nb, dest);
	if (res == -1) {
		if (errno == EMBMDATA) {
			throw PLCException("Too many coils requested");
			//      fprintf(stderr, "Too many coils requested\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when reading coils: ", errno));
			//     fprintf(stderr, "error when reading coils: %s\n",modbus_strerror(errno));
		}

		return false;
	}
	bConnected = true;
	return true;
}


/*The result of reading is stored in dest array as unsigned bytes (8 bits) set to TRUE or FALSE.*/
bool TCP::read_discrete_inputs(int addr, int nb, uint8_t *dest) {
	int res = modbus_read_input_bits(modbus_tcp, addr, nb, dest);
	if (res == -1) {

		if (errno == EMBMDATA) {
			throw PLCException("Too many discrete inputs requested");
			//  fprintf(stderr, "Too many discrete inputs requested\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when reading discrete inputs: ", errno));
			//fprintf(stderr, "Drror when reading discrete inputs: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}
/*The result of the reading is stored in dest array as word values (16 bits).*/
bool TCP::read_holding_registers(int addr, int nb, uint16_t  *dest) {
	int res = modbus_read_registers(modbus_tcp, addr, nb, dest);
	if (res == -1) {
		if (errno == EMBMDATA) {
			throw PLCException("Too many holding registers requested");
			//     fprintf(stderr, "Too many holding registers requested\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when reading holding registers: ", errno));
			//         fprintf(stderr, "Error when reading holding registers: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}

/*The result of reading is stored in dest array as word values (16 bits).*/
bool TCP::read_input_registers(int addr, int nb, uint16_t *dest) {
	int res = modbus_read_input_registers(modbus_tcp, addr, nb, dest);
	if (res == -1) {
		if (errno == EMBMDATA) {
			//       fprintf(stderr, "Too many input registers requested\n");
			throw PLCException("Too many input registers requested");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when reading input registers: ", errno));
			//     fprintf(stderr, "Error when reading input registers: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}
bool TCP::getConnected() {
	return bConnected;
}
/**The status must be set to TRUE or FALSE.*/
bool TCP::write_coil(int addr, int status)
{
	int res = modbus_write_bit(modbus_tcp, addr, status);
	if (res == -1) {
		bConnected = false;
		throw PLCCommException(error_info("Error when writing single coil: ", errno));
		//    fprintf(stderr, "Error when writing single coil: %s\n",modbus_strerror(errno));
		return false;
	}
	bConnected = true;
	return true;
}

/**The src array must contains bytes set to TRUE or FALSE.*/
bool TCP::write_coils(int addr, int nb, const uint8_t *src) {
	int res = modbus_write_bits(modbus_tcp, addr, nb, src);
	if (res == -1) {
		if (errno == EMBMDATA) {
			throw PLCException("Writing too many coils");
			//    fprintf(stderr, "Writing too many coils\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when writing coils: ", errno));
			//    fprintf(stderr, "error when writing coils: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}

bool TCP::write_holding_register(int addr, const uint16_t value) {
	int res = modbus_write_register(modbus_tcp, addr, value);
	if (res == -1) {
		bConnected = false;
		throw PLCCommException(error_info("Error when writing single holding register: ", errno));
		//     fprintf(stderr, "Error when writing single holding register: %s\n",modbus_strerror(errno));
		return false;
	}
	bConnected = true;
	return true;
}

bool TCP::write_holding_registers(int addr, int nb, const uint16_t *src) {
	int res = modbus_write_registers(modbus_tcp, addr, nb, src);
	if (res == -1) {
		if (errno == EMBMDATA) {
			throw PLCException("Writing too many registers");
			//     fprintf(stderr, "Writing too many registers\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when writing holding registers: ", errno));
			//      fprintf(stderr, "error when writing holding registers: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}

bool TCP::write_and_read_holding_registers(int write_addr, int write_nb, const uint16_t *src,
	int read_addr, int read_nb, const uint16_t *dest) {
	int res = write_and_read_holding_registers(write_addr, write_nb, src,
		read_addr, read_nb, dest);
	if (res == -1) {

		if (errno == EMBMDATA) {
			throw PLCException("write and read holding registers:Too many registers requested, Too many registers to write");
			//   fprintf(stderr, "write and read holding registers:Too many registers requested, Too many registers to write\n");
		}
		else {
			bConnected = false;
			throw PLCCommException(error_info("Error when writing and reading holding registers: ", errno));
			//    fprintf(stderr, "error when writing and reading holding registers: %s\n",modbus_strerror(errno));
		}
		return false;
	}
	bConnected = true;
	return true;
}


TCP::~TCP()
{
	close();
	if (modbus_tcp)
		modbus_free(modbus_tcp);
}