/*
 * cli_parser.c
 *
 *  Created on: Nov 22, 2017
 *      Author: spiri
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <msp430.h>
#include "eusci_uartio.h"
#include "cli_parser.h"

static cli_parser_buffer_t cli_buffer;
static const char *argptr[CLI_PARSER_MAX_ARGUMENTS];
static unsigned int arg_head;
static bool ignore_spaces;
static cli_command_t * cli_command_list;

void cli_parser_init()
{
    unsigned int i;

    for (i=0; i < CLI_PARSER_MAX_ARGUMENTS; i++) {
        argptr[i] = (const char *) &(cli_buffer.words[i]);
    }
}

unsigned int cli_parser_process_input(Uartio_t * uart)
{
    unsigned int i, j, cmd_processed_count = 0;

    while (Uartio_available(uart) > 0) {
        char c = (char)Uartio_read(uart);
        /*if (c == '\n') {
            continue; // ignore CR, only trigger on LF
        }*/
        if (c == '"') {
            ignore_spaces ^= true; // Double-quotes lets you specify arguments with spaces inside
            Uartio_write(uart, c);
            continue;
        }
        if (!ignore_spaces && c == ' ') {
            Uartio_write(uart, c);
            if (arg_head > 0) {
                arg_head = 0;
                cli_buffer.argc++;
                if (cli_buffer.argc == CLI_PARSER_MAX_ARGUMENTS) {
                    cli_buffer.argc--;
                } else {
                    cli_buffer.words[cli_buffer.argc][arg_head] = '\0';
                }
            }
            continue;
        }
        if (c == '\x08' || c == '\x7f') { // Backspace
            continue; // Do not print the character, we don't support backspacing; only CTRL-U
        }
        if (c == '\x15') { // CTRL-U, clear line
            // Compute potential total length of line
            i=0;
            for (j=0; j <= cli_buffer.argc; j++) {
                i += 2;
                i += strlen(cli_buffer.words[j]);
            }
            Uartio_writen(uart, "\x0d\x00", 2);
            for (; i > 0; i--) {
                Uartio_write(uart, ' '); // Erase the line
            }
            Uartio_writen(uart, "\x00\x0d\x00", 3);
            cli_buffer.argc = 0;
            arg_head = 0;
            cli_buffer.words[0][0] = '\0';
            continue;
        }
        if (c == '\x12') { // CTRL-L, clear screen
            Uartio_writen(uart, "\x1b\x63\x00", 3);
            cli_buffer.argc = 0;
            arg_head = 0;
            cli_buffer.words[0][0] = '\0';
            continue;
        }
        if (c == '\x0c') { // Not sure what this is, but Putty emits it after ESC-c.
            Uartio_write(uart, c);
            continue;
        }
        if (c == '\r') {
            Uartio_writen(uart, "\x0d\x0a\x00", 3);
            // Process what we have
            if (cli_buffer.argc == 0 && arg_head == 0) {
                continue;  // Nothing to process
            }
            if (arg_head > 0) { // Only do this if the last arg has anything in it; otherwise ignore the last arg
                cli_buffer.argc++;  // during input processing, argc is used as a current pointer,
                                    // but its final purpose to the command processor is a count
            }

            // Walk the command list looking for the words[0] match
            if (cli_command_list != (void *)0) {
                i = 0;
                while (cli_command_list[i].command != (void *)0) {
                    if (cli_strcasecmp(cli_buffer.words[0], cli_command_list[i].command)) {
                        // Found a match!  Process it
                        cli_command_list[i].handler(uart, cli_buffer.argc, argptr, false);
                        cmd_processed_count++;
                        break; // Stop processing command list since we found a match
                    }
                    i++;
                }
            }
            cli_buffer.argc = 0;
            arg_head = 0;
            continue;
        }

        if (arg_head < CLI_PARSER_MAX_ARG_LENGTH) {
            Uartio_write(uart, c);
            // Only add a character if we're under the max arg length; if we exceed, characters are dropped
            // for this argument until the next space... which means we don't clobber the rest of the command arguments
            cli_buffer.words[cli_buffer.argc][arg_head] = c;
            arg_head++;
            cli_buffer.words[cli_buffer.argc][arg_head] = '\0';
        }
    }

    return cmd_processed_count;
}

void cli_parser_reset()
{
    cli_buffer.argc = 0;
    arg_head = 0;
    ignore_spaces = false;
}

void cli_parser_set_command_list(const cli_command_t * list)
{
    cli_command_list = (cli_command_t *)list;
}

bool cli_strcasecmp(const char *s1, const char *s2)
{

    unsigned int l = 0;
    char cmp1, cmp2;

    while (s1[l] != '\0' && s2[l] != '\0') {
        cmp1 = s1[l];
        if (cmp1 >= 'a' && cmp1 <= 'z') { // Ignore case
            cmp1 -= 32;
        }
        cmp2 = s2[l];
        if (cmp2 >= 'a' && cmp2 <= 'z') { // Ignore case
            cmp2 -= 32;
        }
        if (cmp1 != cmp2) {
            return false;
        }
        l++;
    }
    if (s1[l] != '\0' || s2[l] != '\0') { // Unequal sizes
        return false;
    }
    return true;
}

// Assuming out has more than enough space for CLI_PARSER_MAX_ARGUMENTS * CLI_PARSER_MAX_ARG_LENGTH plus room for [] brackets and spaces...
void cli_debug_dump_buffer(char *out)
{
    unsigned int i, j, outhead = 0;

    for (i=0; i <= cli_buffer.argc; i++) {
        out[outhead++] = '[';
        j = 0;
        while (cli_buffer.words[i][j] != '\0') {
            out[outhead++] = cli_buffer.words[i][j];
            j++;
        }
        out[outhead++] = ']';
        out[outhead++] = ' ';
    }
    out[outhead++] = '\0';
}


/* Text interpretation routines; for parsing command arguments */
int32_t cli_atoi(const char *s)
{
    int32_t mult = 1, result = 0;
    int i;
    char c;
    size_t len = strlen(s);

    if (len > 10 || (s[0] == '-' && len > 11)) {
        return 0;  // Invalid
    }

    for (i = len-1; i >= 0; i--) {
        c = s[(unsigned int)i];
        if (i == 0 && c == '-') {
            result = -1 * result;
            continue;
        }
        if (c < '0' || c > '9') {
            return 0;  // Invalid
        }
        c -= '0';
        result += mult * c;
        mult *= 10;
    }

    return result;
}

uint32_t cli_hextoi(const char *s)
{
    int lshft = 0;
    uint32_t result = 0;
    int i;
    char c;
    size_t len = strlen(s);

    if (len > 8) {
        return 0;  // Too long for 32-bits
    }

    for (i = len-1; i >= 0; i--) {
        c = s[(unsigned int)i];
        if (c >= 'a' && c <= 'f') {
            c -= 32;
        }
        if (c >= 'A' && c <= 'F') {
            c -= 7;
        }
        if (c < '0' && c > 63) {
            return 0;  // Invalid
        }
        c -= '0';
        result += (uint32_t)(c) << lshft;
        lshft += 4;
    }

    return result;
}
