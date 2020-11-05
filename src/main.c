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
#include <sys/epoll.h>

#include <libusb.h>

#define LOG_MESSAGES 0
#define VOLUP_COMMAND "pactl set-sink-volume @DEFAULT_SINK@ +1%"
#define VOLDOWN_COMMAND "pactl set-sink-volume @DEFAULT_SINK@ -1%"

// On my system fds up to 9 are taken at start.
#define READINESS_NOTIFICATION_FD 99

#define VENDOR 0x0b05
#define PRODUCT 0x183c
#define INTERFACE_NUM 4

enum ButtonState {
	BUTTON_VOLUP = 1,
	BUTTON_VOLDOWN = 1 << 1,
};

enum OutputState {
	OUTPUT_HEADPHONE = 1 << 4,
	OUTPUT_SPEAKER = 1 << 7,
};

#define BUFSIZE 16

int epoll_fd = -1;

void handle_sigterm(int sig __attribute__((unused)))
{
}

void log_fail(char *func_name)
{
	fprintf(stderr, "%s() failed\n", func_name);
}

void log_fail_with(char *func_name, int error_num)
{
	fprintf(stderr, "%s() failed with: %d\n", func_name, error_num);
}

uint32_t poll_events_to_epoll(short events)
{
	uint32_t epoll_events = 0;
	if (events & POLLIN) {
		epoll_events |= EPOLLIN;
	}
	if (events & POLLOUT) {
		epoll_events |= EPOLLOUT;
	}
	return epoll_events;
}

int add_pollfd_to_epoll(int fd, short events)
{
	struct epoll_event ev = {
		.events = poll_events_to_epoll(events),
		.data = { .fd = fd },
	};
	int errno_saved = errno;
	errno = 0;
	int r = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
	if (r && EEXIST != errno) {
		log_fail_with("epoll_ctl", r);
	}
	errno = errno_saved;
	return r;
}

void process_pollfd_added(int fd, short events,
			  void *user_data __attribute__((unused)))
{
	add_pollfd_to_epoll(fd, events);
}

void process_pollfd_removed(int fd,
			    void *user_data __attribute__((unused)))
{
	int errno_saved = errno;
	errno = 0;
	int r = epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
	// EBADF is *probably* fine.
	if (r && EBADF != errno) {
		log_fail_with("epoll_ctl", r);
	}
	errno = errno_saved;
}

void process_transfer_data(struct libusb_transfer *transfer)
{
	int *bytes_read = transfer->user_data;
	if (LIBUSB_TRANSFER_COMPLETED == transfer->status) {
		*bytes_read = transfer->actual_length;
	} else if (LIBUSB_TRANSFER_CANCELLED == transfer->status) {
		*bytes_read = LIBUSB_TRANSFER_CANCELLED;
		raise(SIGTERM);
	} else {
		*bytes_read = -1;
	}
}

/**
 * @brief Run epoll_wait() and return the value. If it's interrupted by
 * a signal, @p got_signal is set to true.
 */
static int epoll_wait_with_signal(bool *got_signal)
{
	struct epoll_event events[1];
	int errno_saved = errno;
	int r = epoll_wait(epoll_fd, events, 1, -1);
	if (r < 0 && EINTR == errno) {
		*got_signal = true;
	}
	errno = errno_saved;
	return r;
}

static int perform_transfer_blocking(struct libusb_transfer *transfer,
				     libusb_device_handle *handle,
				     unsigned char *buffer, int length)
{
	int bytes_read = 0;
	libusb_fill_interrupt_transfer(
		transfer, handle, INTERFACE_NUM | LIBUSB_ENDPOINT_IN,
		buffer, length, process_transfer_data, &bytes_read, 0);
	int r = libusb_submit_transfer(transfer);
	if (r) {
		log_fail_with("libusb_submit_transfer", r);
		return r;
	}

	bool is_sigint = false;
	do {
		r = epoll_wait_with_signal(&is_sigint);
		if (r < 0 && !is_sigint) {
			log_fail_with("epoll_wait", r);
			return r;
		}

		if (is_sigint) {
			r = libusb_cancel_transfer(transfer);
			if (r) {
				log_fail_with("libusb_cancel_transfer",
					      r);
				return r;
			}
		}

		struct timeval tv = {
			.tv_sec = 0,
			.tv_usec = 0,
		};
		r = libusb_handle_events_timeout_completed(NULL, &tv,
							   NULL);
		if (r) {
			log_fail_with("libusb_handle_events_completed",
				      r);
			return r;
		}

		if (is_sigint) {
			fprintf(stderr, "exiting due to SIGINT\n");
			return -EINTR;
		}
	} while (!bytes_read);

	return bytes_read;
}

int transfer_until_full(struct libusb_transfer *transfer,
			libusb_device_handle *handle,
			unsigned char *buffer, int length)
{
	int r = 0;
	int r_total = 0;
	while (r_total < length) {
		r = perform_transfer_blocking(transfer, handle,
					      buffer + r_total,
					      length - r_total);
		if (r < 0) {
			return r;
		}
		r_total += r;
	}
	return r_total;
}

void process_message(unsigned char *buffer,
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

void process_transfers(libusb_device_handle *handle)
{
	libusb_set_pollfd_notifiers(NULL, process_pollfd_added,
				    process_pollfd_removed, NULL);

	const struct libusb_pollfd **pollfds = libusb_get_pollfds(NULL);
	if (!pollfds) {
		log_fail("libusb_get_pollfds");
		return;
	}
	const struct libusb_pollfd **pollfds_pos = pollfds;
	for (; *pollfds_pos; ++pollfds_pos) {
		const struct libusb_pollfd *pollfd = *pollfds_pos;
		add_pollfd_to_epoll(pollfd->fd, pollfd->events);
	}
	libusb_free_pollfds(pollfds);

	struct libusb_transfer *t = libusb_alloc_transfer(0);
	if (!t) {
		log_fail("libusb_alloc_transfer");
		return;
	}

	unsigned char buffer[BUFSIZE] = { 0 };
	// TODO: request current state on program start.
	enum OutputState output_state = OUTPUT_SPEAKER;
	while (transfer_until_full(t, handle, buffer, BUFSIZE) > 0) {
		process_message(buffer, &output_state);
	}

	libusb_free_transfer(t);
}

int process_input(libusb_device_handle *handle)
{
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

	process_transfers(handle);

	r = libusb_release_interface(handle, INTERFACE_NUM);
	if (r) {
		log_fail("libusb_release_interface");
	}
	return r;
}

int main(void)
{
	signal(SIGTERM, handle_sigterm);

	epoll_fd = epoll_create1(0);
	if (-1 == epoll_fd) {
		log_fail("epoll_create1");
		return EXIT_FAILURE;
	}

	int r = libusb_init(NULL);
	if (r) {
		log_fail("libusb_init");
		goto cleanup_epoll;
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
cleanup_epoll:
	close(epoll_fd);
	if (r) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
