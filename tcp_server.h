/*
 * tcp_server.h
 *
 *  Created on: 16 Aug 2014
 *      Author: pgm
 */

#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_


int tcp_server(const char* host, const char* port,
		int (*interpreter)(FILE* fin, FILE* fout));



#endif /* TCP_SERVER_H_ */
