#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libusb.h>

#define LOG_MESSAGES 0
#define VOLUP_COMMAND "pactl set-sink-volume @DEFAULT_SINK@ +1%"
#define VOLDOWN_COMMAND "pactl set-sink-volume @DEFAULT_SINK@ -1%"

// On my system fds up to 9 are taken at start.
#define READINESS_NOTIFICATION_FD 99

#define VENDOR 0x0b05
#define PRODUCT 0x183c

#define INTERFACE_NUM 4

#define BUFSIZE 16

struct pollfd *libusb_fds = NULL;
size_t libusb_fds_num = 0;

enum ButtonState {
	BUTTON_VOLUP = 1,
	BUTTON_VOLDOWN = 1 << 1,
};

enum OutputState {
	OUTPUT_HEADPHONE = 1 << 4,
	OUTPUT_SPEAKER = 1 << 7,
};

volatile sig_atomic_t got_sigterm = 0;

static void handle_sigterm(int sig)
{
	switch (sig) {
	case SIGTERM:
		got_sigterm = 1;
		break;
	default:
		fprintf(stderr, "got signal: %d\n", sig);
	}
}

static void log_fail(char *func_name)
{
	fprintf(stderr, "%s() failed\n", func_name);
}

static void process_message(unsigned char *buffer,
		     enum OutputState *output_state)
{
	enum ButtonState buttons = (buffer[0] & BUTTON_VOLUP)
				   | (buffer[0] & BUTTON_VOLDOWN);
	if (buttons) {
		if (buttons & BUTTON_VOLUP) {
			if (system(VOLUP_COMMAND)) {
				fprintf(stderr, "system() exited with"
						"non-zero\n");
			}
		}
		if (buttons & BUTTON_VOLDOWN) {
			if (system(VOLDOWN_COMMAND)) {
				fprintf(stderr, "system() exited with"
						"non-zero\n");
			}
		}
	}

	unsigned char output_curr = (buffer[10] & OUTPUT_HEADPHONE)
				    | (buffer[10] & OUTPUT_SPEAKER);
	if (output_curr != *output_state) {
		*output_state = output_curr;
		fprintf(stderr, "output switched to %s\n",
			*output_state == OUTPUT_HEADPHONE ?
				      "headphone" :
				      "speaker");
	}
#if LOG_MESSAGES
	fprintf(stderr, "read bytes: ");
	for (size_t i = 0; i < BUFSIZE; ++i) {
		fprintf(stderr, "%hhx", buffer[i]);
	}
	fprintf(stderr, "\n");
#endif
}

void handle_transfer_completion(struct libusb_transfer *transfer)
{
	int *result = transfer->user_data;
	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
		*result = transfer->actual_length;
		break;
	default:
		*result = -1;
	}
}

int perform_transfer_until_buffer_full(unsigned char *buffer, struct libusb_device_handle *handle, struct libusb_transfer *transfer)
{
	int r = 0;
	int result = -2;
	int total_read = 0;
	while (total_read < BUFSIZE) {
		libusb_fill_interrupt_transfer(transfer, handle, INTERFACE_NUM | LIBUSB_ENDPOINT_IN, buffer + total_read, BUFSIZE - total_read, handle_transfer_completion, &result, 0);

		libusb_submit_transfer(transfer);

		r = poll(libusb_fds, libusb_fds_num, -1);
		if (-1 == r) {
			if (EINTR == errno) {
				fprintf(stderr, "got EINTR\n");
			}
			libusb_cancel_transfer(transfer);
		}

		struct timeval tv = {
			.tv_sec = 0,
			.tv_usec = 0,
		};
		r = libusb_handle_events_timeout_completed(NULL, &tv, NULL);
		if (r == LIBUSB_ERROR_INTERRUPTED && got_sigterm) {
			return -1;
		}

		if (-1 == result) {
			return -1;
		}

		if (-2 != result) {
			total_read += result;
		}
	}

	return total_read;
}

void process_transfers(libusb_device_handle *handle)
{
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);

	unsigned char buffer[BUFSIZE] = { 0 };
	int result = perform_transfer_until_buffer_full(buffer, handle, transfer);
	// TODO: query the device.
	enum OutputState output_state = OUTPUT_SPEAKER;
	while (BUFSIZE == result) {
		process_message(buffer, &output_state);
		result = perform_transfer_until_buffer_full(buffer, handle, transfer);
	}

	if (got_sigterm) {
		fprintf(stderr, "exiting due to SIGTERM\n");
	}

	libusb_free_transfer(transfer);
}

void handle_pollfd_added(int fd, short events, __attribute__((unused))void *user_data)
{
	struct pollfd *new_fds = calloc(libusb_fds_num + 1, sizeof(struct pollfd));
	memcpy(new_fds, libusb_fds, sizeof(struct pollfd) * libusb_fds_num);
	free(libusb_fds);
	libusb_fds = new_fds;
	libusb_fds[libusb_fds_num].fd = fd;
	libusb_fds[libusb_fds_num].events = events;
	++libusb_fds_num;
}

void handle_pollfd_removed(int fd, __attribute__((unused))void *user_data)
{
	struct pollfd *new_fds = calloc(libusb_fds_num - 1, sizeof(struct pollfd));
	size_t i = 0;
	while (libusb_fds[i].fd != fd) {
		++i;
	}
	memcpy(new_fds, libusb_fds, sizeof(struct pollfd) * i);
	if (i < libusb_fds_num - 1) {
		memcpy(new_fds + i, libusb_fds + i + 1, libusb_fds_num - i);
	}
	free(libusb_fds);
	libusb_fds = new_fds;
	--libusb_fds_num;
}

int process_input(libusb_device_handle *handle)
{
	const struct libusb_pollfd **fds = libusb_get_pollfds(NULL);
	size_t fds_count = 0;
	while (fds[fds_count]) {
		++fds_count;
	}
	libusb_fds = calloc(fds_count, sizeof(struct pollfd));
	for (size_t i = 0; i < fds_count; ++i) {
		libusb_fds[i].fd = fds[i]->fd;
		libusb_fds[i].events = fds[i]->events;
	}
	libusb_fds_num = fds_count;
	libusb_free_pollfds(fds);
	libusb_set_pollfd_notifiers(NULL, handle_pollfd_added, handle_pollfd_removed, NULL);

	int r = libusb_set_auto_detach_kernel_driver(handle, 1);
	if (r) {
		log_fail("libusb_set_auto_detach_kernel_driver");
		return r;
	}

	r = libusb_claim_interface(handle, INTERFACE_NUM);
	if (r) {
		log_fail("libusb_claim_interface");
		return r;
	}

	if (fcntl(READINESS_NOTIFICATION_FD, F_GETFD) != -1
	    || errno != EBADF) {
		dprintf(READINESS_NOTIFICATION_FD, "\n");
		close(READINESS_NOTIFICATION_FD);
	} else {
		fprintf(stderr,
			"readiness notification fd is unavailable\n");
	}
	fprintf(stderr, "up and running\n");

	signal(SIGTERM, handle_sigterm);

	sigset_t sigset;
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigset, NULL);

	process_transfers(handle);

	r = libusb_release_interface(handle, INTERFACE_NUM);
	if (r) {
		log_fail("libusb_release_interface");
	}
	return r;
}

int main(void)
{
	sigset_t sigset;
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGTERM);
	sigprocmask(SIG_SETMASK, &sigset, NULL);

	int r = libusb_init(NULL);
	if (r) {
		log_fail("libusb_init");
		return EXIT_FAILURE;
	}

	r = libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL,
			      LIBUSB_LOG_LEVEL_WARNING);
	if (r) {
		log_fail("libusb_set_option");
		goto cleanup;
	}

	libusb_device_handle *handle =
		libusb_open_device_with_vid_pid(NULL, VENDOR, PRODUCT);
	if (!handle) {
		log_fail("libusb_open_device_with_vid_pid");
		goto cleanup;
	}

	r = process_input(handle);

	libusb_close(handle);
cleanup:
	libusb_exit(NULL);
	if (r) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
