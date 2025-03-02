/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/dns.h"

typedef struct TLS_CLIENT_T_ {
    struct altcp_pcb *pcb;
    bool complete;
    int error;
    const char *http_request;
    int timeout;
} TLS_CLIENT_T;

static struct altcp_tls_config *tls_config = NULL;

static err_t tls_client_close(void *arg) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    err_t err = ERR_OK;

    state->complete = true;
    if (state->pcb != NULL) {
        altcp_arg(state->pcb, NULL);
        altcp_poll(state->pcb, NULL, 0);
        altcp_recv(state->pcb, NULL);
        altcp_err(state->pcb, NULL);
        err = altcp_close(state->pcb);
        if (err != ERR_OK) {
            printf("close failed %d, calling abort\n", err);
            altcp_abort(state->pcb);
            err = ERR_ABRT;
        }
        state->pcb = NULL;
    }
    return err;
}

static err_t tls_client_connected(void *arg, struct altcp_pcb *pcb, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    if (err != ERR_OK) {
        printf("connect failed %d\n", err);
        return tls_client_close(state);
    }

    printf("connected to server, sending request\n");
    
    // Add safety check for null PCB
    if (state->pcb == NULL) {
        printf("ERROR: PCB is null when trying to write data\n");
        state->error = PICO_ERROR_GENERIC;
        state->complete = true;
        return ERR_ABRT;
    }
    
    // Add safety check for http_request
    if (state->http_request == NULL) {
        printf("ERROR: HTTP request is null\n");
        state->error = PICO_ERROR_GENERIC;
        return tls_client_close(state);
    }
    
    // Add length safety check
    size_t request_len = strlen(state->http_request);
    if (request_len == 0 || request_len > 10000) {
        printf("ERROR: Invalid request length: %u\n", request_len);
        state->error = PICO_ERROR_GENERIC;
        return tls_client_close(state);
    }
    
    // Add additional check and logging before writing
    printf("Sending request of %u bytes\n", request_len);
    
    // Use a more robust error handling approach for the write operation
    err = altcp_write(state->pcb, state->http_request, request_len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("error writing data, err=%d\n", err);
        state->error = PICO_ERROR_GENERIC;
        return tls_client_close(state);
    }
    
    // Explicitly capture the output of altcp_output() to prevent the panic
    err = altcp_output(state->pcb);
    if (err != ERR_OK) {
        printf("error in altcp_output(), err=%d\n", err);
        state->error = PICO_ERROR_GENERIC;
        return tls_client_close(state);
    }

    return ERR_OK;
}

static err_t tls_client_poll(void *arg, struct altcp_pcb *pcb) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    printf("timed out\n");
    state->error = PICO_ERROR_TIMEOUT;
    return tls_client_close(arg);
}

static void tls_client_err(void *arg, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    printf("tls_client_err %d\n", err);
    tls_client_close(state);
    state->error = PICO_ERROR_GENERIC;
}

static err_t tls_client_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err) {
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;
    if (!p) {
        printf("connection closed\n");
        return tls_client_close(state);
    }

    if (p->tot_len > 0) {
        /* For simplicity this examples creates a buffer on stack the size of the data pending here, 
           and copies all the data to it in one go.
           Do be aware that the amount of data can potentially be a bit large (TLS record size can be 16 KB),
           so you may want to use a smaller fixed size buffer and copy the data to it using a loop, if memory is a concern */
        char buf[p->tot_len + 1];

        pbuf_copy_partial(p, buf, p->tot_len, 0);
        buf[p->tot_len] = 0;

        printf("***\nnew data received from server:\n***\n\n%s\n", buf);

        altcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);

    return ERR_OK;
}

static void tls_client_connect_to_server_ip(const ip_addr_t *ipaddr, TLS_CLIENT_T *state)
{
    err_t err;
    u16_t port = 443;

    printf("connecting to server IP %s port %d\n", ipaddr_ntoa(ipaddr), port);
    err = altcp_connect(state->pcb, ipaddr, port, tls_client_connected);
    if (err != ERR_OK)
    {
        fprintf(stderr, "error initiating connect, err=%d\n", err);
        tls_client_close(state);
    }
}

static void tls_client_dns_found(const char* hostname, const ip_addr_t *ipaddr, void *arg)
{
    if (ipaddr)
    {
        printf("DNS resolving complete\n");
        tls_client_connect_to_server_ip(ipaddr, (TLS_CLIENT_T *) arg);
    }
    else
    {
        printf("error resolving hostname %s\n", hostname);
        tls_client_close(arg);
    }
}

static bool tls_client_open(const char *hostname, void *arg) {
    err_t err;
    ip_addr_t server_ip;
    TLS_CLIENT_T *state = (TLS_CLIENT_T*)arg;

    state->pcb = altcp_tls_new(tls_config, IPADDR_TYPE_ANY);
    if (!state->pcb) {
        printf("failed to create pcb\n");
        return false;
    }

    altcp_arg(state->pcb, state);
    altcp_poll(state->pcb, tls_client_poll, state->timeout * 2);
    altcp_recv(state->pcb, tls_client_recv);
    altcp_err(state->pcb, tls_client_err);

    /* Set SNI */
    mbedtls_ssl_set_hostname(altcp_tls_context(state->pcb), hostname);

    printf("resolving %s\n", hostname);

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();

    err = dns_gethostbyname(hostname, &server_ip, tls_client_dns_found, state);
    if (err == ERR_OK)
    {
        /* host is in DNS cache */
        tls_client_connect_to_server_ip(&server_ip, state);
    }
    else if (err != ERR_INPROGRESS)
    {
        printf("error initiating DNS resolving, err=%d\n", err);
        tls_client_close(state->pcb);
    }

    cyw43_arch_lwip_end();

    return err == ERR_OK || err == ERR_INPROGRESS;
}

// Perform initialisation
static TLS_CLIENT_T* tls_client_init(void) {
    TLS_CLIENT_T *state = calloc(1, sizeof(TLS_CLIENT_T));
    if (!state) {
        printf("failed to allocate state\n");
        return NULL;
    }

    return state;
}

/* Explicitly use extern "C" for the function definition to ensure correct linkage when called from C++ */
#ifdef __cplusplus
extern "C" {
#endif

bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout) {
    // Add input validation
    if (server == NULL || request == NULL || timeout <= 0) {
        printf("Invalid parameters for TLS client test\n");
        return false;
    }

    printf("Starting TLS client with server: %s, timeout: %d ms\n", server, timeout);
    
    /* No CA certificate checking */
    tls_config = altcp_tls_create_config_client(cert, cert_len);
    if (!tls_config) {
        printf("Failed to create TLS config\n");
        return false;
    }

    //mbedtls_ssl_conf_authmode(&tls_config->conf, MBEDTLS_SSL_VERIFY_OPTIONAL);

    TLS_CLIENT_T *state = tls_client_init();
    if (!state) {
        printf("Failed to initialize TLS client\n");
        altcp_tls_free_config(tls_config);
        return false;
    }
    
    state->http_request = request;
    state->timeout = timeout;
    
    if (!tls_client_open(server, state)) {
        printf("Failed to open TLS client connection\n");
        free(state);
        altcp_tls_free_config(tls_config);
        return false;
    }
    
    // Add a timeout for the main loop to prevent infinite waiting
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t max_wait_time = timeout + 180000; // Add 3 minutes to the timeout as buffer (increased from 10 seconds)
    
    // Error recovery tracking
    bool had_errors = false;
    
    // Add a maximum iteration counter to prevent infinite loops
    int iteration_count = 0;
    const int MAX_ITERATIONS = 2000; // Increased from 1000 to allow more iterations for large uploads
    
    printf("TLS client entering processing loop with %d max iterations and %lu ms max wait time\n", 
           MAX_ITERATIONS, (unsigned long)max_wait_time);
    
    while(!state->complete && iteration_count < MAX_ITERATIONS) {
        iteration_count++;
        
        // Log every 50 iterations to help with debugging
        if (iteration_count % 50 == 0) {
            printf("TLS client still processing after %d iterations\n", iteration_count);
        }
        
        // Check if we're approaching the limit
        if (iteration_count >= MAX_ITERATIONS * 0.8) {
            printf("WARNING: Approaching max iterations (%d/%d)\n", iteration_count, MAX_ITERATIONS);
        }
        
        // Check if we've exceeded our max wait time
        uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
        if (elapsed_time > max_wait_time) {
            printf("TLS client operation timed out after %u ms (limit: %u ms)\n", 
                   (unsigned int)elapsed_time, (unsigned int)max_wait_time);
            state->error = PICO_ERROR_TIMEOUT;
            state->complete = true;
            break;
        }
        
        // the following #ifdef is only here so this same example can be used in multiple modes;
        // you do not need it in your code
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        
        // Safe polling with basic error detection
        int poll_result = cyw43_arch_poll();
        if (poll_result < 0) {
            printf("Warning: cyw43_arch_poll returned error %d\n", poll_result);
            had_errors = true;
            sleep_ms(100); // Short sleep to avoid busy-loop on repeated errors
            continue;
        }
        
        // you can poll as often as you like, however if you have nothing else to do you can
        // choose to sleep until either a specified time, or cyw43_arch_poll() has work to do:
        
        // Use a shorter wait time to avoid getting stuck
        absolute_time_t timeout_time = make_timeout_time_ms(100); // Reduced to 100ms from 1000ms
        bool wait_succeeded = cyw43_arch_wait_for_work_until(timeout_time);
        if (!wait_succeeded) {
            printf("Warning: cyw43_arch_wait_for_work_until failed or timed out\n");
            had_errors = true;
            
            // If we're getting repeated errors, count it toward our iteration limit faster
            if (iteration_count > 100) {
                iteration_count += 9; // Make 10 iterations count as 1 (this one + 9 more)
            }
            
            sleep_ms(50); // Very short sleep to recover
            continue;
        }
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(100); // Reduced from 1000ms to avoid long waits
#endif
    }
    
    // Check if we hit the iteration limit
    if (iteration_count >= MAX_ITERATIONS) {
        printf("ERROR: TLS client loop exceeded maximum iterations (%d). Aborting to prevent infinite loop.\n", MAX_ITERATIONS);
        state->error = PICO_ERROR_TIMEOUT;
    }
    
    if (had_errors) {
        printf("TLS client recovered from potential error conditions\n");
    }
    
    int err = state->error;
    printf("TLS client completed with %s (error code: %d, iterations: %d)\n", 
           err == 0 ? "success" : "error", err, iteration_count);
    free(state);
    altcp_tls_free_config(tls_config);
    return err == 0;
}

#ifdef __cplusplus
}
#endif