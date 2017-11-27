/*
 * cli_parser.h
 *
 *  Created on: Nov 22, 2017
 *      Author: spiri
 */

#ifndef CLI_PARSER_H_
#define CLI_PARSER_H_

#include <stdint.h>
#include <stdbool.h>

/* Only one command parser per MCU is supported, and it doesn't care which Uartio_t object
 * you supply... which means you should only supply one module at a time lest you corrupt the
 * parser with data from two or more UARTs at once.
 */

#define CLI_COMMAND_PROMPT_DEFAULT "> "

#define CLI_PARSER_MAX_ARGUMENTS 8
#define CLI_PARSER_MAX_ARG_LENGTH 31
typedef struct {
    char words[CLI_PARSER_MAX_ARGUMENTS][CLI_PARSER_MAX_ARG_LENGTH+1];
    unsigned int argc;
} cli_parser_buffer_t;

// Command handler arguments: UART, ARGC, ARGV, print_help (bool)
typedef void(*CLI_COMMAND)(Uartio_t *, unsigned int, const char **, bool);

typedef struct {
    const char * command;
    CLI_COMMAND handler;
} cli_command_t;

/* Function prototypes */
void cli_parser_init();
unsigned int cli_parser_process_input(Uartio_t *);  // All in one read UART, process commands
void cli_parser_reset();
void cli_parser_set_command_list(const cli_command_t *);
void cli_parser_set_command_prompt(const char *);
void cli_debug_dump_buffer(char *);  // Dumps current cli_buffer arguments surrounded by brackets with spaces between.
bool cli_strcasecmp(const char *, const char *);  // Homemade implementation of strcasecmp(), compares while ignoring case of A-Z/a-z, only returns true if matched
bool cli_strcasehasprefix(const char *s1, const char *pfx); // Checks s1 if it contains pfx as a prefix (case-insensitive match)

/* User text parsing functions */
int32_t cli_atoi(const char *);
uint32_t cli_hextoi(const char *);

#endif /* CLI_PARSER_H_ */
