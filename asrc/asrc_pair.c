/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/mxc_asrc.h>

#include "asrc_pair.h"

#define ASRC_DEVICE     "/dev/mxc_asrc"
#define DMA_MAX_BYTES   (32768)
#define BUF_NUM         (2)
#define ASRC_PADDING_MS	(1)    /* padding in ms for both head and tail */

static void unmap_buffers(asrc_map_area *inp, asrc_map_area *outp, int num)
{
    int i;

    if (inp[0].addr)
        munmap(inp[0].addr, inp[0].len * num);

    for (i = 0; i <= num; i++)
        inp[i].addr = NULL;

    if (outp[0].addr)
        munmap(outp[0].addr, outp[0].len * num);

    for (i = 0; i <= num; i++)
        outp[i].addr = NULL;
}

static int map_buffers(int fd, int num, asrc_map_area *inp, asrc_map_area *outp)
{
    int i;
    int err = 0;
    struct asrc_querybuf buf_info;
    void *addr;

    buf_info.buffer_index = 0;
    if ((err = ioctl(fd, ASRC_QUERYBUF, &buf_info)) < 0)
    {
        fprintf(stderr, "Query ASRC buffers failed\n");
        goto map_fail;
    }

    addr = mmap(NULL, buf_info.input_length * num, PROT_READ | PROT_WRITE,
            MAP_SHARED, fd, buf_info.input_offset);
    if(addr == MAP_FAILED)
    {
        fprintf(stderr, "Mmap ASRC input buffers failed\n");
        err = -1;
        goto map_fail;
    }

    for (i = 0; i < num; i++)
    {
        inp[i].len = buf_info.input_length;
        inp[i].addr = (char *)addr + buf_info.input_length * i;
    }

    addr = mmap(NULL, buf_info.output_length * num, PROT_READ | PROT_WRITE,
            MAP_SHARED, fd, buf_info.output_offset);
    if(addr == MAP_FAILED) {
        fprintf(stderr, "Mmap ASRC output buffers failed\n");
        err = -1;
        goto map_fail;
    }
    for (i = 0; i < num; i++)
    {
        outp[i].len = buf_info.output_length;
        outp[i].addr = (char *)addr + buf_info.output_length * i;
    }

    goto map_end;

map_fail:
    unmap_buffers(inp, outp, num);

map_end:
    return err;
}

static uint32_t get_max_divider(uint32_t x, uint32_t y)
{
    uint32_t t;

    while(y != 0)
    {
        t = x % y;
        x = y;
        y = t;
    }

    return x; 
}

static void calculate_num_den(asrc_pair *pair)
{
    uint32_t div;

    div = get_max_divider(pair->in_rate, pair->out_rate);
    pair->num = pair->in_rate / div;
    pair->den = pair->out_rate / div;
}

static int asrc_start_conversion(asrc_pair *pair)
{
    int err;

    if (pair->is_converting)
        return 0;

    err = ioctl(pair->fd, ASRC_START_CONV, &pair->index);
    if (err < 0)
        fprintf(stderr, "Unable to start ASRC converting %d\n", pair->index);

    pair->is_converting = 1;
    return err;
}

static int asrc_stop_conversion(asrc_pair *pair)
{
    int err;

    if (!pair->is_converting)
        return 0;

    err = ioctl(pair->fd, ASRC_STOP_CONV, &pair->index);
    if (err < 0)
        fprintf(stderr, "Unable to stop ASRC converting %d\n", pair->index);

    pair->is_converting = 0;
    return err;
}

asrc_pair *asrc_pair_create(unsigned int channels, ssize_t in_period_frames,
        ssize_t out_period_frames, unsigned int in_rate, unsigned int out_rate, int type)
{
    int fd;
    int err;
    struct asrc_req req;
    struct asrc_config config;
    asrc_pair *pair = NULL;
    asrc_map_area *in_map, *out_map;
    uint32_t padding_frames = in_rate * ASRC_PADDING_MS / 1000;
    uint32_t dma_buffer_size = (in_period_frames + padding_frames * 2) << 1;

    fd = open(ASRC_DEVICE, O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Unable to open device %s\n", ASRC_DEVICE);
        goto end;
    }

    req.chn_num = channels;
    if ((err = ioctl(fd, ASRC_REQ_PAIR, &req)) < 0)
    {
        fprintf(stderr, "Req ASRC pair failed\n");
        goto close_fd;
    }

    config.pair = req.index;
    config.channel_num = req.chn_num;
    config.dma_buffer_size = dma_buffer_size > DMA_MAX_BYTES ? DMA_MAX_BYTES : dma_buffer_size;
    config.input_sample_rate = in_rate;
    config.output_sample_rate = out_rate;
    config.buffer_num = BUF_NUM;
    config.input_word_width = ASRC_WIDTH_16_BIT;
    config.output_word_width = ASRC_WIDTH_16_BIT;
    config.inclk = INCLK_NONE;
    config.outclk = OUTCLK_ASRCK1_CLK;

    if ((err = ioctl(fd, ASRC_CONFIG_PAIR, &config)) < 0)
    {
        fprintf(stderr, "%s: Config ASRC pair %d failed\n", __func__, req.index);
        goto release_pair;
    }

    in_map = calloc(config.buffer_num, sizeof(asrc_map_area));
    out_map = calloc(config.buffer_num, sizeof(asrc_map_area));
    pair = calloc(1, sizeof(*pair));

    if (!in_map || !out_map || !pair)
        goto release_map;

    if (map_buffers(fd, config.buffer_num, in_map, out_map) < 0)
    {
        fprintf(stderr, "%s: Map ASRC pair %d failed\n", __func__, req.index);
        goto release_map;
    }

    pair->fd = fd;
    pair->type = type;
    pair->index = req.index;
    pair->channels = channels;
    pair->in_rate = in_rate;
    pair->out_rate = out_rate;
    pair->in_period_frames = in_period_frames;
    pair->out_period_frames = out_period_frames;
    pair->buf_size = config.dma_buffer_size;
    pair->buf_num = config.buffer_num;
    pair->in_map = in_map;
    pair->out_map = out_map;
    calculate_num_den(pair);
    
    goto end;

release_map:
    if (in_map)
        free(in_map);
    if (out_map)
        free(out_map);
    if (pair)
    {
        free(pair);
        pair = NULL;
    }

release_pair:
    ioctl(fd, ASRC_RELEASE_PAIR, &req.index);

close_fd:
    close(fd);

end:
    return pair;
}

void asrc_pair_destroy(asrc_pair *pair)
{
    unmap_buffers(pair->in_map, pair->out_map, pair->buf_num);
    free (pair->in_map);
    free (pair->out_map);

    ioctl(pair->fd, ASRC_RELEASE_PAIR, &pair->index);
    close(pair->fd);
    
    free (pair);
}

void asrc_pair_get_ratio(asrc_pair *pair, uint32_t *num, uint32_t *den)
{
    *num = pair->num;
    *den = pair->den;
}

int asrc_pair_set_rate(asrc_pair *pair, ssize_t in_period_frames,
        ssize_t out_period_frames, unsigned int in_rate, unsigned int out_rate)
{
    struct asrc_config config;
    int err;
    uint32_t padding_frames;
    uint32_t dma_buffer_size;

    if (in_rate == pair->in_rate && out_rate == pair->out_rate &&
            in_period_frames == pair->in_period_frames && out_period_frames == pair->out_period_frames)
        return 0;

    padding_frames = in_rate * ASRC_PADDING_MS / 1000;
    dma_buffer_size =  (in_period_frames + padding_frames * 2) << 1;

    unmap_buffers(pair->in_map, pair->out_map, pair->buf_num);
    memset(pair->in_map, 0, pair->buf_num * sizeof(asrc_map_area));
    memset(pair->out_map, 0, pair->buf_num * sizeof(asrc_map_area));

    config.pair = pair->index;
    config.channel_num = pair->channels;
    config.dma_buffer_size = dma_buffer_size > DMA_MAX_BYTES ? DMA_MAX_BYTES : dma_buffer_size;
    config.input_sample_rate = in_rate;
    config.output_sample_rate = out_rate;
    config.buffer_num = pair->buf_num;
    config.input_word_width = ASRC_WIDTH_16_BIT;
    config.output_word_width = ASRC_WIDTH_16_BIT;
    config.inclk = INCLK_NONE;
    config.outclk = OUTCLK_ASRCK1_CLK;

    if ((err = ioctl(pair->fd, ASRC_CONFIG_PAIR, &config)) < 0)
        fprintf(stderr, "%s: Config ASRC pair %d failed\n", __func__, pair->index);
    else
    {
        pair->buf_size = config.dma_buffer_size;
        pair->in_rate = in_rate;
        pair->out_rate = out_rate;
        pair->in_period_frames = in_period_frames;
        pair->out_period_frames = out_period_frames;
        calculate_num_den(pair);

        if ((err = map_buffers(pair->fd, pair->buf_num, pair->in_map, pair->out_map)) < 0)
            fprintf(stderr, "%s: map ASRC pair %d failed\n", __func__, pair->index);
    }

    return err;
}

void asrc_pair_reset(asrc_pair *pair)
{
    int err;

    err = ioctl(pair->fd, ASRC_FLUSH, &pair->index);
    if (err < 0)
        fprintf(stderr, "Unable to flush ASRC %d\n", pair->index);
}

static void asrc_pair_pad(int16_t *dst, int16_t pad_value, uint32_t frames)
{
    while (frames > 0)
    {
        *dst++ = pad_value;
        frames--;
    }
}

static void fill_input_data(asrc_pair *pair, uint32_t *in_frames_left, const int16_t **src,
        uint32_t *src_frames, uint32_t padding_frames, int16_t *dst)
{
    uint32_t buf_frames = pair->buf_size >> 1;
    uint32_t padded_frames, copy_frames, space_left;
    const int16_t *s = *src;
    int16_t *d = dst;
    int16_t pad_head = s[0];
    int16_t pad_tail = s[*src_frames - 1];

    space_left = buf_frames;
    if (*in_frames_left > *src_frames + padding_frames) /* pad head */
    {
        padded_frames = *in_frames_left - *src_frames - padding_frames;
        padded_frames = padded_frames > space_left ? space_left : padded_frames;
        asrc_pair_pad(d, pad_head, padded_frames);
        d += padded_frames;
        *in_frames_left -= padded_frames;
        space_left -= padded_frames;
    }

    if (space_left == 0)
        return;

    if (*in_frames_left <= *src_frames + padding_frames && *in_frames_left > padding_frames) /* fill data */
    {
        copy_frames = *in_frames_left - padding_frames;
        copy_frames = copy_frames > space_left ? space_left : copy_frames;
        memcpy(d, s, copy_frames << 1);
        *src += copy_frames;
        d += copy_frames;
        *in_frames_left -= copy_frames;
        space_left -= copy_frames;
        *src_frames -= copy_frames;
    }

    if (space_left == 0)
        return;

    if (*in_frames_left <= padding_frames)
    {
        padded_frames = *in_frames_left > space_left ? space_left : *in_frames_left;
        asrc_pair_pad(d, pad_tail, padded_frames);
        *in_frames_left -= padded_frames;
    }
}

static void queue_and_wait_result(asrc_pair *pair, const int16_t *src, uint32_t src_frames,
        uint32_t src_frames_total, uint32_t in_padding_frames, uint32_t out_padding_frames,
        int16_t *dst, unsigned int dst_frames)
{
    uint32_t out_frames_left = dst_frames + out_padding_frames;
    const int16_t *s = src;
    int16_t *d = dst;
    int err;
    struct asrc_buffer buffer;
    uint32_t buf_frames = pair->buf_size >> 1;
    uint32_t offset;
    uint32_t copy_frames;
    uint32_t in_frames_left = src_frames_total;
    uint32_t sframes = src_frames;

    while(1)
    {
        if ((err = ioctl(pair->fd, ASRC_POLL_DQ, &buffer)) < 0)
        {
            fprintf(stderr, "%s: Poll buffer of ASRC pair %d failed\n", __func__, pair->index);
            break;
        }

        if (buffer.direction == ASRC_INPUT)
        {
            if (in_frames_left > 0)
            {
                fill_input_data(pair, &in_frames_left, &s, &sframes, in_padding_frames, (int16_t *)pair->in_map[buffer.index].addr);

                if ((err = ioctl(pair->fd, ASRC_Q_INBUF, &buffer)) < 0)
                {
                    fprintf(stderr, "%s: Queue input buffer of ASRC pair %d failed\n", __func__, pair->index);
                    break;
                }
            }
        }
        else
        {
            if (out_frames_left <= dst_frames)  /* all data are valid */
            {
                offset = 0;
                copy_frames = out_frames_left > buf_frames ? buf_frames : out_frames_left;
            }
            else if (out_frames_left >= dst_frames + buf_frames) /* all data are padding */
            {
                offset = buf_frames;
                copy_frames = 0;
            }
            else
            {
                offset = out_frames_left - dst_frames;
                copy_frames = buf_frames > out_frames_left ? out_frames_left - offset : buf_frames - offset;
            }

            if (copy_frames > 0)
            {
                memcpy(d, (char*)pair->out_map[buffer.index].addr + (offset << 1), copy_frames << 1);
                d += copy_frames;
            }

            out_frames_left -= copy_frames + offset;

            if ((err = ioctl(pair->fd, ASRC_Q_OUTBUF, &buffer)) < 0)
            {
                fprintf(stderr, "%s: Queue outbuf of ASRC pair %d failed\n", __func__, pair->index);
                break;
            }
            
            if (out_frames_left == 0) /* all data processed */
                break;
        }
    }
}

void asrc_pair_convert_s16(asrc_pair *pair, const int16_t *src, unsigned int src_frames,
        int16_t *dst, unsigned int dst_frames)
{
    uint32_t buf_frames = pair->buf_size >> 1;
    uint32_t min_padding_frames = pair->in_rate * ASRC_PADDING_MS / 1000;
    int buf_num_needed = (src_frames + min_padding_frames * 2 + (buf_frames - 1)) / buf_frames;
    uint32_t padding_frames = (buf_frames * buf_num_needed - src_frames) / 2; /* half at head and half at tail */
    int buf_num_prefill = buf_num_needed > pair->buf_num ? pair->buf_num : buf_num_needed;
    int i, err;
    const int16_t *s;
    uint32_t sframes;
    int16_t *d;
    uint32_t in_frames_left = src_frames + padding_frames * 2; /* we need pad at head and tail */
    struct asrc_buffer inbuf, outbuf;
    uint32_t out_frames, out_padding_frames;

    s = src;
    sframes = src_frames;

    /* prefill input buffer */
    for (i = 0; i < buf_num_prefill; i++)
    {
        d = (int16_t *)pair->in_map[i].addr;
        fill_input_data(pair, &in_frames_left, &s, &sframes, padding_frames, d);
    }

    /* queue prefilled buffer */
    for (i = 0; i < buf_num_prefill; i++)
    {
        inbuf.index = i;
        inbuf.length = pair->buf_size;
        if ((err = ioctl(pair->fd, ASRC_Q_INBUF, &inbuf)) < 0)
        {
            fprintf(stderr, "%s: Queue inbuf of ASRC pair %d failed\n", __func__, pair->index);
            return;
        }
    }

    for (i = 0; i < pair->buf_num; i++)
    {
        outbuf.index = i;
        outbuf.length = pair->buf_size;
        if ((err = ioctl(pair->fd, ASRC_Q_OUTBUF, &outbuf)) < 0)
        {
            fprintf(stderr, "%s: Queue outbuf of ASRC pair %d failed\n", __func__, pair->index);
            return;
        }
    }

    asrc_start_conversion(pair);

    out_frames = src_frames * pair->den / pair->num;
    out_padding_frames = padding_frames * pair->den / pair->num; 
    out_frames = out_frames < dst_frames ? out_frames : dst_frames;

    d = dst + dst_frames - out_frames;

    queue_and_wait_result(pair, s, sframes, in_frames_left, padding_frames, out_padding_frames, d, out_frames);

    asrc_stop_conversion(pair);
    asrc_pair_reset(pair);
}

