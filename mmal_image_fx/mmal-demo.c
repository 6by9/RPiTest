/**
 * mmal demo utility for testing decode, vout and deinterlace filter
 *
 * build with:
 * gcc -lbcm_host -lmmal -lmmal_core -lpthread -o mmal-demo mmal-demo.c
 *
 * using deinterlace filter:
 * gcc -DDEINTERLACE -lbcm_host -lmmal -lmmal_core -lpthread -o mmal-demo mmal-demo.c
 *
 * cross build:
 * arm-linux-gcc \
 *   --sysroot=/path/to/your/sysroot \
 *   -DDEINTERLACE -lbcm_host -lmmal -lmmal_core -lpthread -o mmal-demo mmal-demo.c
 **/
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>
#include <sys/time.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_default_components.h>

#define FPS_THRESHOLD 2000.0

#define MMAL_COMPONENT_DEFAULT_DEINTERLACE "vc.ril.image_fx"

struct data_t {
	MMAL_COMPONENT_T *dec;
	MMAL_PORT_T *dec_input;
	MMAL_PORT_T *dec_output;
	MMAL_POOL_T *dec_input_pool;
	MMAL_QUEUE_T *decoded;

	MMAL_COMPONENT_T *vout;
	MMAL_PORT_T *vout_input;
	MMAL_POOL_T *vout_input_pool;

	MMAL_ES_FORMAT_T *format;

	MMAL_COMPONENT_T *deinterlace;
	MMAL_PORT_T *deinterlace_input;
	MMAL_PORT_T *deinterlace_output;
	MMAL_POOL_T *deinterlace_input_pool;
	MMAL_QUEUE_T *deinterlaced;

	pthread_mutex_t mutex;
	pthread_cond_t cond;
	pthread_t worker;

	pthread_t deinterlace_worker;
};

static volatile sig_atomic_t aborted = 0;

static void on_signal(int sig);
double millisecs(void);
static uint32_t align(uint32_t x, uint32_t y);
static int change_output_format(struct data_t *data);
static void *vout_worker(void *p);
static void *deinterlace_worker(void *p);

static void dec_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void dec_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void dec_output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

static void deinterlace_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void deinterlace_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void deinterlace_output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

static void vout_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
static void vout_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

static void* pool_allocator_alloc(void *context, uint32_t size);
static void pool_allocator_free(void *context, void *mem);

int main(int argc, char *argv[]) {
	struct data_t data;
	double t1, t2;
	int64_t frames = 0;
	FILE *src;
	MMAL_PARAMETER_BOOLEAN_T error_concealment;
	MMAL_BUFFER_HEADER_T *buffer;
	size_t len;
	MMAL_STATUS_T status;
	int ret = EXIT_SUCCESS;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	memset(&data, 0, sizeof(struct data_t));

	pthread_mutex_init(&data.mutex, NULL);
	pthread_cond_init(&data.cond, NULL);

	if (argc < 2) {
		printf("Specify path to h264 file\n");
		ret = EXIT_FAILURE;
		goto out;
	}

	src = fopen(argv[1], "r");
	if (!src) {
		printf("Failed to open %s\n", argv[1]);
		ret = EXIT_FAILURE;
		goto out;
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &data.dec);
	if (status != MMAL_SUCCESS) {
		printf("Failed to create MMAL decoder component %s (status=%"PRIx32" %s)\n", MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data.dec->control->userdata = (struct MMAL_PORT_USERDATA_T *)&data;
	status = mmal_port_enable(data.dec->control, dec_control_port_cb);
	if (status != MMAL_SUCCESS) {
		printf("Failed to enable decoder control port %s (status=%"PRIx32" %s)\n", data.dec->control->name, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data.dec_input = data.dec->input[0];
	data.dec_input->userdata = (struct MMAL_PORT_USERDATA_T *)&data;
	data.dec_input->format->encoding = MMAL_ENCODING_H264;

	error_concealment.hdr.id = MMAL_PARAMETER_VIDEO_DECODE_ERROR_CONCEALMENT;
	error_concealment.hdr.size = sizeof(MMAL_PARAMETER_BOOLEAN_T);
	error_concealment.enable = MMAL_FALSE;
	status = mmal_port_parameter_set(data.dec_input, &error_concealment.hdr);
	if (status != MMAL_SUCCESS)
		printf("Failed to disable error concealment on %s (status=%"PRIx32" %s)\n", data.dec_input->name, status, mmal_status_to_string(status));

	status = mmal_port_format_commit(data.dec_input);
	if (status != MMAL_SUCCESS) {
		printf("Failed to commit format for decoder input port %s (status=%"PRIx32" %s)\n", data.dec_input->name, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}
	data.dec_input->buffer_size = data.dec_input->buffer_size_recommended;
	data.dec_input->buffer_num = data.dec_input->buffer_num_recommended;

	status = mmal_port_enable(data.dec_input, dec_input_port_cb);
	if (status != MMAL_SUCCESS) {
		printf("Failed to enable decoder input port %s (status=%"PRIx32" %s)\n", data.dec_input->name, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data.dec_output = data.dec->output[0];
	data.dec_output->userdata = (struct MMAL_PORT_USERDATA_T *)&data;

	status = mmal_port_enable(data.dec_output, dec_output_port_cb);
	if (status != MMAL_SUCCESS) {
		printf("Failed to enable decoder output port %s (status=%"PRIx32" %s)\n", data.dec_output->name, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	status = mmal_component_enable(data.dec);
	if (status != MMAL_SUCCESS) {
		printf("Failed to enable decoder component %s (status=%"PRIx32" %s)\n", data.dec->name, status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data.dec_input_pool = mmal_pool_create_with_allocator(data.dec_input->buffer_num, data.dec_input->buffer_size, data.dec_input, pool_allocator_alloc, pool_allocator_free);
	if(!data.dec_input_pool) {
		printf("Failed to create pool for decoder input port (%d, %s)\n", status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	t1 = millisecs();
	while(!aborted) {
      buffer = mmal_queue_get(data.dec_input_pool->queue);
      if (buffer) {
         mmal_buffer_header_reset(buffer);
         buffer->cmd = 0;
         buffer->pts = 1;
         buffer->length = fread(buffer->data, 1, buffer->alloc_size, src);
         // buffer->length != buffer->alloc_size => EOF

         printf("main - dec_input: buffer %p, len %d, send to port %p\n", buffer, buffer->length, data.dec_input);
         status = mmal_port_send_buffer(data.dec_input, buffer);
         if (status != MMAL_SUCCESS) {
            printf("Failed send buffer to decoder input port (%d, %s)\n", status, mmal_status_to_string(status));
            ret = EXIT_FAILURE;
            goto out;
         }
      }

		if (data.format && !data.vout_input_pool) {
			if (change_output_format(&data) < 0)
				goto out;
			else {
				pthread_create(&data.worker, NULL, vout_worker, &data);
#ifdef DEINTERLACE
				pthread_create(&data.deinterlace_worker, NULL, deinterlace_worker, &data);
#endif
			}
		}

		if (data.vout_input_pool) {
			buffer = mmal_queue_get(data.vout_input_pool->queue);

			if (buffer) {
				mmal_buffer_header_reset(buffer);
				buffer->cmd = 0;

#ifdef DEINTERLACE
            printf("main: Send buffer %p from pool to deinterlace output port %p\n", buffer, data.deinterlace_output);
				status = mmal_port_send_buffer(data.deinterlace_output, buffer);
#else
            printf("main: Send buffer %p from pool to decoder output port %p\n", buffer, data.dec_output);
				status = mmal_port_send_buffer(data.dec_output, buffer);
#endif
				if (status != MMAL_SUCCESS) {
					printf("Failed send buffer to decoder output port (%d, %s)\n", status, mmal_status_to_string(status));
					ret = EXIT_FAILURE;
					goto out;
				}
			}
		}

		if (data.deinterlace_input_pool) {
			buffer = mmal_queue_get(data.deinterlace_input_pool->queue);

			if (buffer) {
				mmal_buffer_header_reset(buffer);
				buffer->cmd = 0;

            printf("Send buffer %p from pool to decoder output port %p\n", buffer, data.dec_output);
				status = mmal_port_send_buffer(data.dec_output, buffer);
				if (status != MMAL_SUCCESS) {
					printf("Failed send buffer to decoder output port (%d, %s)\n", status, mmal_status_to_string(status));
					ret = EXIT_FAILURE;
					goto out;
				}
			}
		}

		++frames;
		t2 = millisecs();
		if(t2 - t1 >= FPS_THRESHOLD) {
			printf("fps: %lf\n", 1000 * frames / (t2 - t1));
			frames = 0;
			t1 = t2;
		}

		usleep(10000);
	}

out:
	pthread_cond_destroy(&data.cond);
	pthread_mutex_destroy(&data.mutex);

	if (data.dec) {
		mmal_component_disable(data.dec);
		mmal_port_disable(data.dec->control);
	}

	if (data.dec_input)
		mmal_port_disable(data.dec_input);

	if (data.dec_output)
		mmal_port_disable(data.dec_output);

	if (data.vout) {
		mmal_component_disable(data.vout);
		mmal_port_disable(data.vout->control);
	}

	if (data.vout_input)
		mmal_port_disable(data.vout_input);

	if (data.decoded)
		mmal_queue_destroy(data.decoded);

	if (data.dec_input_pool)
		mmal_pool_destroy(data.dec_input_pool);

	if (data.vout_input_pool)
		mmal_pool_destroy(data.vout_input_pool);

	if (data.vout)
		mmal_component_release(data.vout);

	if (data.deinterlace)
		mmal_component_release(data.deinterlace);

	if (data.dec)
		mmal_component_release(data.dec);

	if (data.format)
		mmal_format_free(data.format);

	return ret;
}

static void on_signal(int sig)
{
	if(aborted) {
		abort();
	}

	aborted = 1;
}

double millisecs(void) {
	struct timeval tv;
	double result = 0;

	if(gettimeofday(&tv, NULL) == 0) {
		result = (tv.tv_sec * 1000) + (tv.tv_usec / 1000.0);
	}

	return result;
}

static uint32_t align(uint32_t x, uint32_t y) {
	uint32_t mod = x % y;

	if(mod == 0) {
		return x;
	}
	else {
		return x + y - mod;
	}
}

static int change_output_format(struct data_t *data)
{
	MMAL_STATUS_T status;
	int ret = 0;

	status = mmal_port_disable(data->dec_output);
	if (status != MMAL_SUCCESS) {
		printf("Failed to disable decoder output port (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	mmal_format_full_copy(data->dec_output->format, data->format);
	status = mmal_port_format_commit(data->dec_output);
	if (status != MMAL_SUCCESS) {
		printf("Failed to commit output format (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->dec_output->buffer_num = 40;
	data->dec_output->buffer_size = data->dec_output->buffer_size_min;
	status = mmal_port_enable(data->dec_output, dec_output_port_cb);
	if (status != MMAL_SUCCESS) {
		printf("Failed to enable output port (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

#ifdef DEINTERLACE
	/* Create deinterlace filter */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_DEINTERLACE, &data->deinterlace);
	if(status != MMAL_SUCCESS) {
		printf("Failed to create deinterlace component %s (%x, %s)\n", MMAL_COMPONENT_DEFAULT_DEINTERLACE, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}
   {
      MMAL_PARAMETER_IMAGEFX_PARAMETERS_T imfx_param = {{MMAL_PARAMETER_IMAGE_EFFECT_PARAMETERS,sizeof(imfx_param)},
            MMAL_PARAM_IMAGEFX_DEINTERLACE_DOUBLE, 0, {0}};
      mmal_port_parameter_set(data->deinterlace->output[0], &imfx_param.hdr);
   }
   
	data->deinterlace->control->userdata = (struct MMAL_PORT_USERDATA_T *)data;
	status = mmal_port_enable(data->deinterlace->control, deinterlace_control_port_cb);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable deinterlace control port %s (%x, %s)\n", data->vout->control->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->deinterlace_input = data->deinterlace->input[0];
	data->deinterlace_input->userdata = (struct MMAL_PORT_USERDATA_T *)data;
	/* FIXME: Probably this buffer_num is not sane, something low like 3
	 * or so should work well, I'd expect */
	mmal_format_full_copy(data->deinterlace_input->format, data->format);
	data->deinterlace_input->buffer_num = data->dec_output->buffer_num;
	status = mmal_port_format_commit(data->deinterlace_input);
	if (status != MMAL_SUCCESS) {
		printf("Failed to commit deinterlace intput format (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	status = mmal_port_enable(data->deinterlace_input, deinterlace_input_port_cb);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable deinterlace input port %s (%d, %s)\n", data->deinterlace_input->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->deinterlace_output = data->deinterlace->output[0];
	data->deinterlace_output->userdata = (struct MMAL_PORT_USERDATA_T *)data;
	/* FIXME: Do we have to configure output format for the deinterlace
	 * filter? Or will it auto-populate with a matching format for input
	 * data? */
	mmal_format_full_copy(data->deinterlace_output->format, data->format);
	data->deinterlace_output->buffer_num = data->dec_output->buffer_num;
	status = mmal_port_format_commit(data->deinterlace_output);
	if (status != MMAL_SUCCESS) {
		printf("Failed to commit deinterlace outtput format (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	status = mmal_port_enable(data->deinterlace_output, deinterlace_output_port_cb);
   printf("data->deinterlace_input enabled with %d buffers\n", data->deinterlace_input->buffer_num);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable deinterlacer output port %s (%d, %s)\n", data->deinterlace_output->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	status = mmal_component_enable(data->deinterlace);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable deinterlace component %s (%d, %s)\n", data->deinterlace->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->deinterlace_input_pool = mmal_pool_create_with_allocator(data->deinterlace_output->buffer_num, data->deinterlace_input->buffer_size, data->deinterlace_input, pool_allocator_alloc, pool_allocator_free);
	if(!data->deinterlace_input_pool) {
		printf("Failed to create pool for deinterlace input port (%d, %s)\n", status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data->deinterlaced = mmal_queue_create();
#endif

	/* Create video renderer */
	/* FIXME: Should we move this to format change of deinterlace plugin?
	 * */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &data->vout);
	if(status != MMAL_SUCCESS) {
		printf("Failed to create vout component %s (%x, %s)\n", MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->vout->control->userdata = (struct MMAL_PORT_USERDATA_T *)data;
	status = mmal_port_enable(data->vout->control, vout_control_port_cb);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable vout control port %s (%x, %s)\n", data->vout->control->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->vout_input = data->vout->input[0];
	data->vout_input->userdata = (struct MMAL_PORT_USERDATA_T *)data;
	mmal_format_full_copy(data->vout_input->format, data->format);
   data->vout_input->buffer_num = data->dec_output->buffer_num;
	status = mmal_port_format_commit(data->vout_input);
	if (status != MMAL_SUCCESS) {
		printf("Failed to commit vout intput format (status=%"PRIx32" %s)\n", status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	status = mmal_port_enable(data->vout_input, vout_input_port_cb);
	if(status != MMAL_SUCCESS) {
		printf("Failed to vout enable input port %s (%d, %s)\n", data->vout_input->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	status = mmal_component_enable(data->vout);
	if(status != MMAL_SUCCESS) {
		printf("Failed to enable vout component %s (%d, %s)\n", data->vout->name, status, mmal_status_to_string(status));
		ret = -1;
		goto out;
	}

	data->vout_input_pool = mmal_pool_create_with_allocator(data->vout_input->buffer_num, data->vout_input->buffer_size, data->vout_input, pool_allocator_alloc, pool_allocator_free);
	if(!data->vout_input_pool) {
		printf("Failed to create pool for vout input port (%d, %s)\n", status, mmal_status_to_string(status));
		ret = EXIT_FAILURE;
		goto out;
	}

	data->decoded = mmal_queue_create();

out:
    return ret;
}

static void *vout_worker(void *p)
{
	struct data_t *data = (struct data_t *)p;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!aborted) {
#ifdef DEINTERLACE
		buffer = mmal_queue_wait(data->deinterlaced);
#else
		buffer = mmal_queue_wait(data->decoded);
#endif
	printf("vout_worker: buffer %p, length %d to port %p\n", buffer, buffer->length, data->vout_input);
		mmal_port_send_buffer(data->vout_input, buffer);
	}

	return NULL;
}

static void *deinterlace_worker(void *p)
{
	struct data_t *data = (struct data_t *)p;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!aborted) {
		buffer = mmal_queue_wait(data->decoded);
	printf("deinterlace_worker: buffer %p, len %d, send to port %p\n", buffer, buffer->length, data->deinterlace_input);
		mmal_port_send_buffer(data->deinterlace_input, buffer);
	}

	return NULL;
}

static void dec_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	MMAL_STATUS_T status;

	if (buffer->cmd == MMAL_EVENT_ERROR) {
		status = *(uint32_t *)buffer->data;
		printf("Decoder MMAL error %"PRIx32" \"%s\"", status, mmal_status_to_string(status));
	}

	mmal_buffer_header_release(buffer);
}

static void dec_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	mmal_buffer_header_release(buffer);
}

static void dec_output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	struct data_t *data = (struct data_t *)port->userdata;
	MMAL_EVENT_FORMAT_CHANGED_T *fmt;
	MMAL_ES_FORMAT_T *format;

	if (buffer->cmd == 0) {
	printf("dec_output_port_cb: buffer %p, len %d\n", buffer, buffer->length);
		if (buffer->length > 0) {
			mmal_queue_put(data->decoded, buffer);
		}
		else {
			mmal_buffer_header_release(buffer);
		}
	}
	else if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED) {
		fmt = mmal_event_format_changed_get(buffer);

		format = mmal_format_alloc();
		mmal_format_full_copy(format, fmt->format);
		format->encoding = MMAL_ENCODING_OPAQUE;
		data->format = format;
		mmal_buffer_header_release(buffer);
	}
	else {
		mmal_buffer_header_release(buffer);
	}
}

static void deinterlace_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	mmal_buffer_header_release(buffer);
}

static void deinterlace_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	printf("deinterlace_input_port_cb: buffer %p, len %d\n", buffer, buffer->length);
	mmal_buffer_header_release(buffer);
}

static void deinterlace_output_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	struct data_t *data = (struct data_t *)port->userdata;
	MMAL_EVENT_FORMAT_CHANGED_T *fmt;
	MMAL_ES_FORMAT_T *format;

	if (buffer->cmd == 0) {
	printf("deinterlace_output_worker: buffer %p, len %d\n", buffer, buffer->length);
		if (buffer->length > 0) {
			mmal_queue_put(data->deinterlaced, buffer);
		} else {
			mmal_buffer_header_release(buffer);
		}
	} else {
		mmal_buffer_header_release(buffer);
	}
}

static void vout_control_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	mmal_buffer_header_release(buffer);
}

static void vout_input_port_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	printf("vout_input_port_cb: buffer %p, len %d\n", buffer, buffer->length);
	mmal_buffer_header_release(buffer);
}

static void* pool_allocator_alloc(void *context, uint32_t size)
{
	return mmal_port_payload_alloc((MMAL_PORT_T *)context, size);
}

static void pool_allocator_free(void *context, void *mem)
{
	mmal_port_payload_free((MMAL_PORT_T *)context, (uint8_t *)mem);
}
