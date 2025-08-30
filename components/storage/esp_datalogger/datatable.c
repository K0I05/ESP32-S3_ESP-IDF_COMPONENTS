/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file datatable.c
 *
 * ESP-IDF library for DATA-TABLE
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/datatable.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief Data-table context structure definition.
 */
typedef struct datatable_context_s {
    const char*                         name;                       /*!< data-table textual name, maximum of 15 characters */
    datatable_data_storage_types_t      data_storage_type;          /*!< data-table data storage type, defines handling of records when the data-table is full, set when data-table is created */
    uint16_t                            sampling_count;             /*!< data-table data sampling count seed number */
    time_into_interval_handle_t         sampling_tii_handle;        /*!< data-table sampling time-into-interval handle */
    time_into_interval_handle_t         processing_tii_handle;      /*!< data-table processing time-into-interval handle */
    uint16_t                            record_id;                  /*!< data-table record identifier seed number */
    uint8_t                             columns_count;              /*!< data-table column count seed number, this number should not exceed the column size*/
    uint8_t                             columns_size;               /*!< data-table column array size, static, set when data-table is created */
    datatable_column_t**                columns;                    /*!< array of data-table columns */
    datatable_process_t**               processes;                  /*!< array of data-table column processes, same size as column array */
    datatable_buffer_t**                buffers;                    /*!< array of data-table column buffers, same size as column array */
    uint16_t                            rows_count;                 /*!< data-table row count seed number, this number should not exceed the row size*/
    uint16_t                            rows_size;                  /*!< data-table row array size, static, set when data-table is created */
    datatable_row_t**                   rows;                       /*!< array of data-table rows */
    uint16_t                            samples_maximum_size;       /*!< data-table column samples size maximum, this is calculated from the sampling and processing intervals */
    uint16_t                            hash_code;                  /*!< hash-code of the data-table handle */
    SemaphoreHandle_t                   mutex_handle;
    datatable_event                     event_handler;
} datatable_context_t;

/*
* static constant declarations
*/
static const char *TAG = "data-table";

/**
 * @brief Concatenates the `append` string to the `base` string.
 * 
 * @param base String base.
 * @param append String to append to the base.
 * @return const char* `append` string concatenated to the `base` string.
 */
static inline const char* datatable_concat(const char* base, const char* append) {
    char *res = malloc(strlen(base) + strlen(append) + 1);
    strcpy(res, base);
    strcat(res, append);
    return res;
}

/**
 * @brief Gets a 16-bit hash-code utilizing epoch timestamp as the seed.
 * 
 * @return uint16_t 16-bit hash-code.
 */
static inline uint16_t datatable_get_hash_code(void) {
    uint16_t seed_hash = (uint16_t)time_into_interval_get_epoch_timestamp();
    return ((seed_hash>>16) ^ (seed_hash)) & 0xFFFF;
}

/**
 * @brief Converts degrees to radians.
 * 
 * @param degree degrees to convert.
 * @return double converted degrees to radians.
 */
static inline double datatable_degrees_to_radians(double degree) {
    return degree * (M_PI / 180);
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param radian radian to convert.
 * @return double converted radians to degrees.
 */
static inline double datatable_radians_to_degrees(double radian) {
    return radian * (180 / M_PI);
}

//https://github.com/skeeto/scratch/blob/master/misc/float16.c
/**
 * @brief Encodes a double to an int16_t data-type.
 * 
 * @param value double value to encode.
 * @return uint16_t encoded double value.
 */
static inline uint16_t datatable_encode_double(double f) {
    uint64_t b;
    memcpy(&b, &f, 8);
    int s = (b>>48 & 0x8000);
    int e = (b>>52 & 0x07ff) - 1023;
    int m = (b>>42 & 0x03ff);
    int t = !!(b && 0xffffffffffff);

    if (e == -1023) {
        // input is denormal, round to zero
        e = m = 0;
    } else if (e < -14) {
        // convert to denormal
        if (-14 - e > 10) {
            m = 0;
        } else {
            m |= 0x400;
            m >>= -14 - e - 1;
            m = (m>>1) + (m&1);  // round
        }
        e = 0;
    } else if (e > +16) {
        // NaN / overflow to infinity
        m &= t << 9;  // canonicalize to quiet NaN
        e = 31;
    } else {
        e += 15;
    }

    return s | e<<10 | m;
}

/**
 * @brief Decodes a uint16_t to a double data-type.
 * 
 * @param value uint16_t value to decode.
 * @return double decoded uint16_t value.
 */
static inline double datatable_decode_uint16(uint16_t x) {
    int s = (x     & 0x8000);
    int e = (x>>10 & 0x001f) - 15;
    int m = (x     & 0x03ff);

    switch (e) {
    case -15: if (!m) {
                  e = 0;
              } else {
                  // convert from denormal
                  e += 1023 + 1;
                  while (!(m&0x400)) {
                      e--;
                      m <<= 1;
                  }
                  m &= 0x3ff;
              }
              break;
    case +16: m = !!m << 9;  // canonicalize to quiet NaN
              e = 2047;
              break;
    default:  e += 1023;
    }

    uint64_t b = (uint64_t)s<<48 |
                 (uint64_t)e<<52 |
                 (uint64_t)m<<42;
    double f;
    memcpy(&f, &b, 8);
    return f;
}


/**
 * @brief Converts a data-table column process-type enumerator to a short abbreviated textual string.  The shortened and 
 * abbreviated textual string of the process-type uses a julian convention e.g. `DATATABLE_COLUMN_PROCESS_SMP` represents a 
 * data-table column process-type as a sample and will output `Smp` for the short abbreviated textual string.
 * 
 * For full textual representation of the data-table column process-type, see `datatable_process_type_to_string` function.
 * 
 * @param process_type Data-table column process-type enumerator to convert to a short abbreviated textual string.
 * @return char* Data-table column process-type as a short abbreviated textual string.
 */
static inline const char* datatable_process_type_to_short_string(const datatable_column_process_types_t process_type) {
    /* handle process type to short string */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            return "Smp";
        case DATATABLE_COLUMN_PROCESS_AVG:
            return "Avg";
        case DATATABLE_COLUMN_PROCESS_MIN:
            return "Min";
        case DATATABLE_COLUMN_PROCESS_MAX:
            return "Max";
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            return "Min-TS";
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            return "Max-TS";
        default:
            return "-";
    }
}

/**
 * @brief Converts a data-table column process-type enumerator to a textual string.  The textual string of the process-type 
 * uses a julian convention e.g. `DATATABLE_COLUMN_PROCESS_MIN_TS` represents a data-table column process-type 
 * as a sample and will output `Minimum-TimeStamp` for the textual string.
 * 
 * For a short abbreviated textual representation of the data-table column process-type, see `datatable_process_type_to_short_string` 
 * function.
 * 
 * @param process_type Data-table column process-type enumerator to convert to a textual string.
 * @return char* Data-table column process-type as a textual string.
 */
static inline const char* datatable_process_type_to_string(const datatable_column_process_types_t process_type) {
    /* handle process type to string */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            return "Sample";
        case DATATABLE_COLUMN_PROCESS_AVG:
            return "Average";
        case DATATABLE_COLUMN_PROCESS_MIN:
            return "Minimum";
        case DATATABLE_COLUMN_PROCESS_MAX:
            return "Maximum";
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            return "Minimum-TimeStamp";
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            return "Maximum-TimeStamp";
        default:
            return "-";
    }
}

static inline const char* datatable_data_type_to_string(const datatable_column_data_types_t data_type) {
    /* handle data-type to string  */
    switch(data_type) {
        case DATATABLE_COLUMN_DATA_ID:
            return "ID";
        case DATATABLE_COLUMN_DATA_TS:
            return "TS";
        case DATATABLE_COLUMN_DATA_VECTOR:
            return "Vector";
        case DATATABLE_COLUMN_DATA_BOOL:
            return "Bool";
        case DATATABLE_COLUMN_DATA_FLOAT:
            return "Float";
        case DATATABLE_COLUMN_DATA_INT16:
            return "Int16";
        default:
            return "-";
    }
}

/**
 * @brief Serializes data-logger time interval type enumerator to a textual string for json formating.
 * 
 * @param interval_type Data-logger time interval type enumerator to serialize.
 * @return char* Serialized data-logger time interval type as a textual string.
 */
static inline const char* datatable_json_serialize_interval_type(const time_into_interval_types_t interval_type) {
    /* normalize  */
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            return "second";
        case TIME_INTO_INTERVAL_MIN:
            return "minute";
        case TIME_INTO_INTERVAL_HR:
            return "hour";
        default:
            return "-";
    }
}

/**
 * @brief Serializes data-table column data-type enumerator to a textual string for json formating.
 * 
 * @param data_type Data-table column data-type enumerator to serialize.
 * @return char* Serialized data-table column data-type as a textual string.
 */
static inline const char* datatable_json_serialize_column_data_type(const datatable_column_data_types_t data_type) {
    /* handle data-type to string  */
    switch(data_type) {
        case DATATABLE_COLUMN_DATA_ID:
            return "id";
        case DATATABLE_COLUMN_DATA_TS:
            return "ts";
        case DATATABLE_COLUMN_DATA_VECTOR:
            return "vector";
        case DATATABLE_COLUMN_DATA_BOOL:
            return "bool";
        case DATATABLE_COLUMN_DATA_FLOAT:
            return "float";
        case DATATABLE_COLUMN_DATA_INT16:
            return "int16";
        default:
            return "-";
    }
}

/**
 * @brief Serializes data-table column process-type enumerator to a textual string for json formating.
 * 
 * @param process_type Data-table column process-type enumerator to serialize.
 * @return char* Serialized data-table column process-type as a textual string.
 */
static inline const char* datatable_json_serialize_process_type(const datatable_column_process_types_t process_type) {
    /* handle process type to string */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            return "sample";
        case DATATABLE_COLUMN_PROCESS_AVG:
            return "average";
        case DATATABLE_COLUMN_PROCESS_MIN:
            return "minimum";
        case DATATABLE_COLUMN_PROCESS_MAX:
            return "maximum";
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            return "minimum-timestamp";
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            return "maximum-timestamp";
        default:
            return "-";
    }
}

/**
 * @brief Concatenates the `process_type` string to the column `base_name` string.
 * 
 * @param base_name Base column name.
 * @param process_type Column process type.
 * @return const char* Column name with concatenated `process_type` string.
 */
static inline const char* datatable_concat_column_name(const char* base_name, const datatable_column_process_types_t process_type) {
    const char* pt_str = datatable_process_type_to_short_string(process_type);
    const char* us_str = "_";
    char* res = malloc(strlen(base_name) + strlen(pt_str) + strlen(us_str) + 1);
    strcpy(res, base_name);
    strcat(res, us_str);
    strcat(res, pt_str);
    return res;
}

/**
 * @brief Invokes data-table event when the data-table event handler is configured.
 * 
 * @param datatable_context Data-table context descriptor.
 * @param event_type Data-table event type.
 * @param message Data-table event message.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_invoke_event(datatable_context_t *const datatable_context, const datatable_event_types_t event_type, const char* message) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate event handler */
    if(!datatable_context->event_handler) return ESP_ERR_INVALID_STATE;

    /* initialize event structure */
    const datatable_event_t dt_event = {
        .type    = event_type,
        .message = message
    };

    /* invoke event */
    datatable_context->event_handler(datatable_context, dt_event);

    return ESP_OK;
}

/**
 * @brief Gets the data-table column sample size maximum.  The data-table column sample size maximum is a whole number that
 * represents the total number of samples that can be pushed onto the stack for analytical processing which is 
 * defined by the process-type setting for each column configured in the data-table.
 * 
 * As an example, if the data-table sampling task schedule event occurs every 10-seconds and the data-table 
 * processing event occurs every minute, data-table column data buffer created for analytical processing will be
 * sized to store a maximum of 6 samples.  If the data-table column data buffer created for analytical processing 
 * is processed with less than 6 samples, the data-table record is skipped, and logged as a skipped record  
 * processing event for the data-table.
 * 
 * @param datatable_context Data-table context descriptor.
 * @param size Data-table column sample size of maximum samples that can be stored.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_get_column_samples_maximum_size(datatable_context_t *const datatable_context, uint16_t *size) {
    time_into_interval_types_t smp_interval_type;
    uint16_t smp_interval_period;
    time_into_interval_types_t prc_interval_type;
    uint16_t prc_interval_period;
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* get sampling interval type and period */
    ESP_GOTO_ON_ERROR( time_into_interval_get_interval(datatable_context->sampling_tii_handle, &smp_interval_type, &smp_interval_period), err, TAG, "get sampling interval type and period failed");

    /* get processing interval type and period */
    ESP_GOTO_ON_ERROR( time_into_interval_get_interval(datatable_context->processing_tii_handle, &prc_interval_type, &prc_interval_period), err, TAG, "get processing interval type and period failed");

    /* normalize sampling and processing periods to seconds, and set delta interval */
    uint64_t sampling_interval   = time_into_interval_normalize_interval_to_sec(smp_interval_type, smp_interval_period);
    uint64_t processing_interval = time_into_interval_normalize_interval_to_sec(prc_interval_type, prc_interval_period);
    int64_t  interval_delta      = processing_interval - sampling_interval;

    //ESP_LOGD(TAG, "datatable_get_column_data_buffer_size (delta %lli): processing_interval(%llu) / sampling_interval(%llu)", interval_delta, processing_interval, sampling_interval);

    /* validate sampling and processing intervals to avoid dividing by 0 */

    /* validate sampling interval */
    ESP_GOTO_ON_FALSE((sampling_interval > 0), ESP_ERR_INVALID_SIZE, err, TAG, "sampling interval is out of range, sampling interval cannot be 0, get column data buffer size failed");

    /* validate processing interval */
    ESP_GOTO_ON_FALSE((processing_interval > 0), ESP_ERR_INVALID_SIZE, err, TAG, "processing interval is out of range, processing interval cannot be 0, get column data buffer size failed");

    /* validate sampling and processing interval delta - processing interval must be greater than the sampling interval */
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_SIZE, err, TAG, "sampling and processing intervals are out of range, sampling interval cannot be greater than the processing interval, get column data buffer size failed");

    /* calculate data-table column data buffer sample size */
    *size = (uint16_t)(processing_interval / sampling_interval);

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;

    err:
        xSemaphoreGive(datatable_context->mutex_handle);
        return ret;
}

/**
 * @brief Checks if the data-table column exist by column index.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] index Data-table column index to check if it exist.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and column does not exist.
 */
static inline esp_err_t datatable_column_exist(datatable_context_t *const datatable_context, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate index */
    ESP_RETURN_ON_FALSE((index < datatable_context->columns_count), ESP_ERR_INVALID_ARG, TAG, "index is out of range, get column failed");

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Checks if the data-table is full (i.e. number of rows matches configured rows size).
 * 
 * @param datatable_context Data-table context descriptor.
 * @param full True when data-table is full, otherwise, false.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_is_full(datatable_context_t *const datatable_context, bool *full) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate if the data-table is full and set output parameter */
    if(datatable_context->rows_count >= datatable_context->rows_size) {
        *full = true;
    } else {
        *full = false;
    }

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Frees a data-table buffer entity and subentities.
 * 
 * @param buffer Data-table buffer entity to free.
 */
static inline void datatable_free_buffer(datatable_buffer_t* buffer, const uint16_t samples_size, const datatable_column_data_types_t data_type) {
    if(buffer == NULL) return;
    for(uint16_t i = 0; i < samples_size; i++) {
        switch(data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                break;
            case DATATABLE_COLUMN_DATA_TS:
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                if(buffer->vector_samples[i] != NULL) free(buffer->vector_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                if(buffer->bool_samples[i] != NULL) free(buffer->bool_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                if(buffer->float_samples[i] != NULL) free(buffer->float_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                if(buffer->int16_samples[i] != NULL) free(buffer->int16_samples[i]);
                break;    
        }
    }
    if(buffer->vector_samples != NULL) free(buffer->vector_samples);
    if(buffer->bool_samples   != NULL) free(buffer->bool_samples);
    if(buffer->float_samples  != NULL) free(buffer->float_samples);
    if(buffer->int16_samples  != NULL) free(buffer->int16_samples);
    free(buffer);
}

/**
 * @brief Frees a data-table row entity and subentities.
 * 
 * @param row Data-table row entity to free.
 */
static inline void datatable_free_row(datatable_row_t* row, const uint8_t columns_size) {
    if(row == NULL) return;
    if(row->data_columns != NULL) {
        for(uint8_t i = 0; i < columns_size; i++) {
            if(row->data_columns[i] != NULL) free(row->data_columns[i]);
        }
        free(row->data_columns);
    }
    free(row);
}

/**
 * @brief Pops the top data-table row and shifts the index of remaining rows up by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_context Data-table context descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_rows(datatable_context_t *const datatable_context) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* TODO - use goto statements to give semaphore on error and free-up resources */

    /* validate memory availability for temporary data-table rows */
    datatable_row_t** dt_rows = (datatable_row_t**)calloc(datatable_context->rows_size, sizeof(datatable_row_t*));
    ESP_RETURN_ON_FALSE( dt_rows, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table rows, data-table fifo rows failed" );

    /* perform a deep copy of the data-table handle rows */
    for(uint16_t i = 0; i < datatable_context->rows_size; i++) {
        /* validate memory availability for temporary data-table row */
        dt_rows[i] = (datatable_row_t*)calloc(1, sizeof(datatable_row_t));
        ESP_RETURN_ON_FALSE( dt_rows[i], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table row, data-table fifo rows failed" );

        /* validate memory availability for temporary data-table row data columns */
        dt_rows[i]->data_columns = (datatable_row_data_column_t**)calloc(datatable_context->columns_size, sizeof(datatable_row_data_column_t*));
        ESP_RETURN_ON_FALSE( dt_rows[i]->data_columns, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table row data columns, data-table fifo rows failed" );

        /* perform a deep copy of the data-table row data columns */
        for(uint8_t ii = 0; ii < datatable_context->columns_size; ii++) {
            /* validate memory availability for temporary data-table row data column */
            dt_rows[i]->data_columns[ii] = (datatable_row_data_column_t*)calloc(1, sizeof(datatable_row_data_column_t));
            ESP_RETURN_ON_FALSE( dt_rows[i]->data_columns[ii], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table row data column, data-table fifo rows failed" );

            /* handle data-table column data-type */
            switch(datatable_context->columns[ii]->data_type) {
                case DATATABLE_COLUMN_DATA_ID:
                    dt_rows[i]->data_columns[ii]->id_data = datatable_context->rows[i]->data_columns[ii]->id_data;
                    break;
                case DATATABLE_COLUMN_DATA_TS:
                    dt_rows[i]->data_columns[ii]->ts_data = datatable_context->rows[i]->data_columns[ii]->ts_data;
                    break;
                case DATATABLE_COLUMN_DATA_VECTOR:
                    dt_rows[i]->data_columns[ii]->vector_data = datatable_context->rows[i]->data_columns[ii]->vector_data;
                    break;
                case DATATABLE_COLUMN_DATA_BOOL:
                    dt_rows[i]->data_columns[ii]->bool_data = datatable_context->rows[i]->data_columns[ii]->bool_data;
                    break;
                case DATATABLE_COLUMN_DATA_FLOAT:
                    dt_rows[i]->data_columns[ii]->float_data = datatable_context->rows[i]->data_columns[ii]->float_data;
                    break;
                case DATATABLE_COLUMN_DATA_INT16:
                    dt_rows[i]->data_columns[ii]->int16_data = datatable_context->rows[i]->data_columns[ii]->int16_data;
                    break;
            }
        }

        /* free data-table handle row */
        datatable_free_row(datatable_context->rows[i], datatable_context->columns_size);
    }

    /* reinstate data-table rows but shift index by one to pop the first row (fifo) */
    for(uint16_t i = 0; i < datatable_context->rows_size - 1; i++) {
        /* validate memory availability for data-table handle row */
        datatable_context->rows[i] = (datatable_row_t*)calloc(1, sizeof(datatable_row_t));
        ESP_RETURN_ON_FALSE( datatable_context->rows[i], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle row, data-table fifo rows failed" );

        /* validate memory availability for data-table handle row data columns */
        datatable_context->rows[i]->data_columns = (datatable_row_data_column_t**)calloc(datatable_context->columns_size, sizeof(datatable_row_data_column_t*));
        ESP_RETURN_ON_FALSE( datatable_context->rows[i]->data_columns, ESP_ERR_NO_MEM, TAG, "no memory for data-table handle row data columns, data-table fifo rows failed" );

        /* reinstate data-table row data columns */
        for(uint8_t ii = 0; ii < datatable_context->columns_size; ii++) {
            /* validate memory availability for data-table handle row data column */
            datatable_context->rows[i]->data_columns[ii] = (datatable_row_data_column_t*)calloc(1, sizeof(datatable_row_data_column_t));
            ESP_RETURN_ON_FALSE( datatable_context->rows[i]->data_columns[ii], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle row data column, data-table fifo rows failed" );

            /* handle data-table column data-type */
            switch(datatable_context->columns[ii]->data_type) {
                case DATATABLE_COLUMN_DATA_ID:
                    datatable_context->rows[i]->data_columns[ii]->id_data = dt_rows[i + 1]->data_columns[ii]->id_data;
                    break;
                case DATATABLE_COLUMN_DATA_TS:
                    datatable_context->rows[i]->data_columns[ii]->ts_data = dt_rows[i + 1]->data_columns[ii]->ts_data;
                    break;
                case DATATABLE_COLUMN_DATA_VECTOR:
                    datatable_context->rows[i]->data_columns[ii]->vector_data = dt_rows[i + 1]->data_columns[ii]->vector_data;
                    break;
                case DATATABLE_COLUMN_DATA_BOOL:
                    datatable_context->rows[i]->data_columns[ii]->bool_data = dt_rows[i + 1]->data_columns[ii]->bool_data;
                    break;
                case DATATABLE_COLUMN_DATA_FLOAT:
                    datatable_context->rows[i]->data_columns[ii]->float_data = dt_rows[i + 1]->data_columns[ii]->float_data;
                    break;
                case DATATABLE_COLUMN_DATA_INT16:
                    datatable_context->rows[i]->data_columns[ii]->int16_data = dt_rows[i + 1]->data_columns[ii]->int16_data;
                    break;
            }
        }

        /* free temporary data-table row */
        datatable_free_row(dt_rows[i], datatable_context->columns_size);
    }

    /* free temporary data-table last row */
    datatable_free_row(dt_rows[datatable_context->rows_size - 1], datatable_context->columns_size);

    /* free temporary data-table rows */
    if(dt_rows != NULL) free(dt_rows);

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_FIFO_ROWS, "rows FIFO operation was successful");
    }

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}


/**
 * @brief Resets data-table rows, this is full reset, all data is deleted and data-table row array is re-initialized per configured row size.
 * 
 * @param datatable_context Data-table context descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_rows(datatable_context_t *const datatable_context) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* reset row attributes */
    datatable_context->rows_count = 0;

    /* free all rows */
    for(uint16_t r = 0; r < datatable_context->rows_size; r++) {
        datatable_free_row(datatable_context->rows[r], datatable_context->columns_size);
    }

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_RESET_ROWS, "rows reset operation was successful");
    }

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Resets data-table column data buffer by column index.
 * 
 * @param datatable_context Data-table context descriptor.
 * @param index Index of data-table column to reset column data buffer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_data_buffer(datatable_context_t *const datatable_context, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, data-table reset column data buffer failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    for(uint16_t i = 0; i < datatable_context->processes[index]->samples_size; i++) {
        switch(datatable_context->columns[index]->data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                break;
            case DATATABLE_COLUMN_DATA_TS:
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                if(datatable_context->buffers[index]->vector_samples[i]) free(datatable_context->buffers[index]->vector_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                if(datatable_context->buffers[index]->bool_samples[i]) free(datatable_context->buffers[index]->bool_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                if(datatable_context->buffers[index]->float_samples[i]) free(datatable_context->buffers[index]->float_samples[i]);
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                if(datatable_context->buffers[index]->int16_samples[i]) free(datatable_context->buffers[index]->int16_samples[i]);
                break;
        }
    }

    datatable_context->processes[index]->samples_count = 0;

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Resets data-table column data buffer for configured columns.
 * 
 * @param datatable_context Data-table context descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_data_buffers(datatable_context_t *const datatable_context) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* handle each data-table column */
    for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
        /* pop and shift data buffer stack */
        ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_context, ci),TAG, "reset column data buffer for data-table reset column data buffers failed" );
    }

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* reset sampling count */
    datatable_context->sampling_count = 0;

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_RESET_SAMPLES, "buffer samples reset operation was successful");
    }

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Pops the top data-table data buffer by column index and shifts the index of remaining 
 * samples up by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_context Data-table context descriptor.
 * @param index Data-table column index to FIFO data buffer to pop.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_data_buffer(datatable_context_t *const datatable_context, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, data-table fifo column data buffer failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);
    
    /* validate memory availability for temporary data-table buffers */
    datatable_buffer_t** dt_buffers = (datatable_buffer_t**)calloc(datatable_context->columns_size, sizeof(datatable_buffer_t*));
    ESP_RETURN_ON_FALSE( dt_buffers, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffers, data-table fifo data buffer failed" );

    for(uint8_t i = 0; i < datatable_context->columns_size; i++) {
        /* validate memory availability for temporary data-table buffer */
        dt_buffers[i] = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
        ESP_RETURN_ON_FALSE( dt_buffers[i], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffers, data-table fifo data buffer failed" );

        switch(datatable_context->columns[i]->data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                break;
            case DATATABLE_COLUMN_DATA_TS:
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                /* validate memory availability for temporary data-table buffer samples */
                dt_buffers[i]->vector_samples = (datatable_vector_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_vector_column_data_type_t*));
                ESP_RETURN_ON_FALSE( dt_buffers[i]->vector_samples, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size; ii++) {
                    /* validate memory availability for temporary data-table buffer sample */
                    dt_buffers[i]->vector_samples[ii] = (datatable_vector_column_data_type_t*)calloc(1, sizeof(datatable_vector_column_data_type_t));
                    ESP_RETURN_ON_FALSE( dt_buffers[i]->vector_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer sample, data-table fifo data buffer failed" );

                    /* copy values to temporary data-table buffer sample */
                    dt_buffers[i]->vector_samples[ii]->value_ts = datatable_context->buffers[i]->vector_samples[ii]->value_ts;
                    dt_buffers[i]->vector_samples[ii]->value_uc = datatable_context->buffers[i]->vector_samples[ii]->value_uc;
                    dt_buffers[i]->vector_samples[ii]->value_vc = datatable_context->buffers[i]->vector_samples[ii]->value_vc;
                }
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                /* validate memory availability for temporary data-table buffer samples */
                dt_buffers[i]->bool_samples = (datatable_bool_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_bool_column_data_type_t*));
                ESP_RETURN_ON_FALSE( dt_buffers[i]->bool_samples, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size; ii++) {
                    /* validate memory availability for temporary data-table buffer sample */
                    dt_buffers[i]->bool_samples[ii] = (datatable_bool_column_data_type_t*)calloc(1, sizeof(datatable_bool_column_data_type_t));
                    ESP_RETURN_ON_FALSE( dt_buffers[i]->bool_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    dt_buffers[i]->bool_samples[ii]->value = datatable_context->buffers[i]->bool_samples[ii]->value;
                }
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                /* validate memory availability for temporary data-table buffer samples */
                dt_buffers[i]->float_samples = (datatable_float_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_float_column_data_type_t*));
                ESP_RETURN_ON_FALSE( dt_buffers[i]->float_samples, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size; ii++) {
                    /* validate memory availability for temporary data-table buffer sample */
                    dt_buffers[i]->float_samples[ii] = (datatable_float_column_data_type_t*)calloc(1, sizeof(datatable_float_column_data_type_t));
                    ESP_RETURN_ON_FALSE( dt_buffers[i]->float_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    dt_buffers[i]->float_samples[ii]->value_ts = datatable_context->buffers[i]->float_samples[ii]->value_ts;
                    dt_buffers[i]->float_samples[ii]->value    = datatable_context->buffers[i]->float_samples[ii]->value;
                }
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                /* validate memory availability for temporary data-table buffer samples */
                dt_buffers[i]->int16_samples = (datatable_int16_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_int16_column_data_type_t*));
                ESP_RETURN_ON_FALSE( dt_buffers[i]->int16_samples, ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size; ii++) {
                    /* validate memory availability for temporary data-table buffer sample */
                    dt_buffers[i]->int16_samples[ii] = (datatable_int16_column_data_type_t*)calloc(1, sizeof(datatable_int16_column_data_type_t));
                    ESP_RETURN_ON_FALSE( dt_buffers[i]->int16_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for temporary data-table buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    dt_buffers[i]->int16_samples[ii]->value_ts = datatable_context->buffers[i]->int16_samples[ii]->value_ts;
                    dt_buffers[i]->int16_samples[ii]->value    = datatable_context->buffers[i]->int16_samples[ii]->value;
                }
                break;
        }
    
        /* free data-table handle buffer */
        datatable_free_buffer(datatable_context->buffers[i], datatable_context->processes[i]->samples_size, datatable_context->columns[i]->data_type);
    }

    for(uint8_t i = 0; i < datatable_context->columns_size; i++) {
        /* validate memory availability for data-table handle buffer */
        datatable_context->buffers[i] = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
        ESP_RETURN_ON_FALSE( datatable_context->buffers[i], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffers, data-table fifo data buffer failed" );

        switch(datatable_context->columns[i]->data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                break;
            case DATATABLE_COLUMN_DATA_TS:
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                /* validate memory availability for data-table handle buffer samples */
                datatable_context->buffers[i]->vector_samples = (datatable_vector_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_vector_column_data_type_t*));
                ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->vector_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size - 1; ii++) {
                    /* validate memory availability for data-table handle buffer sample */
                    datatable_context->buffers[i]->vector_samples[ii] = (datatable_vector_column_data_type_t*)calloc(1, sizeof(datatable_vector_column_data_type_t));
                    ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->vector_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer sample, data-table fifo data buffer failed" );

                    /* copy values to temporary data-table buffer sample */
                    datatable_context->buffers[i]->vector_samples[ii]->value_ts = dt_buffers[i]->vector_samples[ii + 1]->value_ts;
                    datatable_context->buffers[i]->vector_samples[ii]->value_uc = dt_buffers[i]->vector_samples[ii + 1]->value_uc;
                    datatable_context->buffers[i]->vector_samples[ii]->value_vc = dt_buffers[i]->vector_samples[ii + 1]->value_vc;
                }
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                /* validate memory availability for data-table handle buffer samples */
                datatable_context->buffers[i]->bool_samples = (datatable_bool_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_bool_column_data_type_t*));
                ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->bool_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size - 1; ii++) {
                    /* validate memory availability for data-table handle buffer sample */
                    datatable_context->buffers[i]->bool_samples[ii] = (datatable_bool_column_data_type_t*)calloc(1, sizeof(datatable_bool_column_data_type_t));
                    ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->bool_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    datatable_context->buffers[i]->bool_samples[ii]->value = dt_buffers[i]->bool_samples[ii + 1]->value;
                }
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                /* validate memory availability for data-table handle buffer samples */
                datatable_context->buffers[i]->float_samples = (datatable_float_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_float_column_data_type_t*));
                ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->float_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size - 1; ii++) {
                    /* validate memory availability for data-table handle buffer sample */
                    datatable_context->buffers[i]->float_samples[ii] = (datatable_float_column_data_type_t*)calloc(1, sizeof(datatable_float_column_data_type_t));
                    ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->float_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    datatable_context->buffers[i]->float_samples[ii]->value_ts = dt_buffers[i]->float_samples[ii + 1]->value_ts;
                    datatable_context->buffers[i]->float_samples[ii]->value    = dt_buffers[i]->float_samples[ii + 1]->value;
                }
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                /* validate memory availability for data-table handle buffer samples */
                datatable_context->buffers[i]->int16_samples = (datatable_int16_column_data_type_t**)calloc(datatable_context->processes[i]->samples_size, sizeof(datatable_int16_column_data_type_t*));
                ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->int16_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer samples, data-table fifo data buffer failed" );

                for(uint16_t ii = 0; ii < datatable_context->processes[i]->samples_size - 1; ii++) {
                    /* validate memory availability for data-table handle buffer sample */
                    datatable_context->buffers[i]->int16_samples[ii] = (datatable_int16_column_data_type_t*)calloc(1, sizeof(datatable_int16_column_data_type_t));
                    ESP_RETURN_ON_FALSE( datatable_context->buffers[i]->int16_samples[ii], ESP_ERR_NO_MEM, TAG, "no memory for data-table handle buffer sample, data-table fifo data buffer failed" );
                    
                    /* copy values to temporary data-table buffer sample */
                    datatable_context->buffers[i]->int16_samples[ii]->value_ts = dt_buffers[i]->int16_samples[ii + 1]->value_ts;
                    datatable_context->buffers[i]->int16_samples[ii]->value    = dt_buffers[i]->int16_samples[ii + 1]->value;
                }
                break;
        }

        /* free temporary data-table buffer */
        datatable_free_buffer(dt_buffers[i], datatable_context->processes[i]->samples_size, datatable_context->columns[i]->data_type);
    }

    /* free temporary data-table buffer last */
    datatable_free_buffer(dt_buffers[datatable_context->columns_size - 1], datatable_context->processes[datatable_context->columns_size - 1]->samples_size, datatable_context->columns[datatable_context->columns_size - 1]->data_type);

    /* free temporary data-table buffer  */
    if(dt_buffers != NULL) free (dt_buffers);

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);
    
    return ESP_OK;
}

/**
 * @brief Pops the first data-table data buffers for configured columns and shifts the index remaining 
 * samples by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_context Data-table context descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_data_buffers(datatable_context_t *const datatable_context) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* handle each data-table column */
    for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
        /* pop and shift data buffer stack */
        ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_context, ci),TAG, "fifo column data buffer for data-table fifo column data buffers failed" );
    }

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_FIFO_SAMPLES, "buffer samples FIFO operation was successful");
    }

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;
}

/**
 * @brief Processes data-table vector data-type data buffer samples on the stack by column based on the column index provided.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] index Data-table column index to process.
 * @param[out] value_uc Data-table column data buffer processed u-component value.
 * @param[out] value_vc Data-table column data buffer processed v-component value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_vector_data_buffer(datatable_context_t *const datatable_context, const uint8_t index, float *value_uc, float *value_vc, time_t *value_ts) {
    double tmp_ew_vector    = NAN;
    double tmp_ns_vector    = NAN;
    double tmp_ew_avg       = NAN;
    double tmp_ns_avg       = NAN;
    double tmp_atan2_uc     = NAN;
    double tmp_uc_avg       = NAN;
    double tmp_uc_value     = NAN;
    double tmp_vc_value     = NAN;
    time_t tmp_ts_value     = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range for process vector data buffer failed" );

    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_VECTOR, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process vector data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_context->processes[index]->samples_count != datatable_context->processes[index]->samples_size) {
        /* set default data */
        *value_uc = tmp_uc_value;
        *value_vc = tmp_vc_value;
        *value_ts = tmp_ts_value;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_context->processes[index]->process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value_uc = datatable_context->buffers[index]->vector_samples[0]->value_uc;
            *value_vc = datatable_context->buffers[index]->vector_samples[0]->value_vc;
            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            /*
            * http://www.webmet.com/met_monitoring/622.html
            * https://www.scadacore.com/2014/12/19/average-wind-direction-and-wind-speed/
            */
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_ew_vector = sin( datatable_degrees_to_radians(datatable_context->buffers[index]->vector_samples[s]->value_uc) ) * datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    tmp_ns_vector = cos( datatable_degrees_to_radians(datatable_context->buffers[index]->vector_samples[s]->value_uc) ) * datatable_context->buffers[index]->vector_samples[s]->value_vc;
                } else {
                    tmp_ew_vector += sin( datatable_degrees_to_radians(datatable_context->buffers[index]->vector_samples[s]->value_uc) ) * datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    tmp_ns_vector += cos( datatable_degrees_to_radians(datatable_context->buffers[index]->vector_samples[s]->value_uc) ) * datatable_context->buffers[index]->vector_samples[s]->value_vc;
                }
            }

            // average in radians
            tmp_ew_avg = (tmp_ew_vector / datatable_context->processes[index]->samples_count) * -1;
            tmp_ns_avg = (tmp_ns_vector / datatable_context->processes[index]->samples_count) * -1;

            // average u-component in degrees
            tmp_atan2_uc = atan2(tmp_ew_avg, tmp_ns_avg);
            tmp_uc_avg = datatable_radians_to_degrees(tmp_atan2_uc);

            // apply correction as specified: http://www.webmet.com/met_monitoring/622.html
            if(tmp_uc_avg > 180.0) {
                tmp_uc_avg -= 180.0;
            } else if(tmp_uc_avg < 180.0) {
                tmp_uc_avg += 180.0;
            }
            *value_uc = tmp_uc_avg;

            // average v-component
            *value_vc = sqrt((tmp_ew_avg * tmp_ew_avg) + (tmp_ns_avg * tmp_ns_avg));

            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                    tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                } else {
                    if(datatable_context->buffers[index]->vector_samples[s]->value_vc < tmp_vc_value) {
                        tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                        tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                    tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                } else {
                    if(datatable_context->buffers[index]->vector_samples[s]->value_vc > tmp_vc_value) {
                        tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                        tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_ts_value = datatable_context->buffers[index]->vector_samples[s]->value_ts;
                    tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                    tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                } else {
                    if(datatable_context->buffers[index]->vector_samples[s]->value_vc < tmp_vc_value) {
                        tmp_ts_value = datatable_context->buffers[index]->vector_samples[s]->value_ts;
                        tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                        tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_ts_value = datatable_context->buffers[index]->vector_samples[s]->value_ts;
                    tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                    tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                } else {
                    if(datatable_context->buffers[index]->vector_samples[s]->value_vc > tmp_vc_value) {
                        tmp_ts_value = datatable_context->buffers[index]->vector_samples[s]->value_ts;
                        tmp_uc_value = datatable_context->buffers[index]->vector_samples[s]->value_uc;
                        tmp_vc_value = datatable_context->buffers[index]->vector_samples[s]->value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
    }

    return ESP_OK;
}

/**
 * @brief Processes data-table boolean data-type data buffer samples on the stack by column based on the column index provided.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_bool_data_buffer(datatable_context_t *const datatable_context, const uint8_t index, bool *value) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range for process bool data buffer failed" );

    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_BOOL, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process bool data buffer failed" );

    /* validate column process-type */
    ESP_RETURN_ON_FALSE( datatable_context->processes[index]->process_type == DATATABLE_COLUMN_PROCESS_SMP, ESP_ERR_INVALID_ARG, TAG, "column process-type is incorrect for process bool data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_context->processes[index]->samples_count != datatable_context->processes[index]->samples_size) {
        /* set default data */
        *value = false;

        return ESP_ERR_INVALID_SIZE;
    }

    /* set value from sample */
    *value = datatable_context->buffers[index]->bool_samples[0]->value;

    return ESP_OK;
}

/**
 * @brief Processes data-table float data-type data buffer samples on the stack by column based on the column index provided.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_float_data_buffer(datatable_context_t *const datatable_context, const uint8_t index, float *value, time_t *value_ts) {
    float tmp_value = NAN;
    time_t tmp_ts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range for process float data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_FLOAT, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process float data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_context->processes[index]->samples_count != datatable_context->processes[index]->samples_size) {
        /* set default data */
        *value = tmp_value;
        *value_ts = tmp_ts;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_context->processes[index]->process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value = datatable_context->buffers[index]->float_samples[0]->value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                } else {
                    tmp_value += datatable_context->buffers[index]->float_samples[s]->value;
                }
            }
            *value = tmp_value / datatable_context->processes[index]->samples_count;
            *value_ts = tmp_ts;
            ESP_LOGW(TAG, "datatable_process_float_data_buffer(column-index: %u) data-count: %u data-avg: %.2f", index, datatable_context->processes[index]->samples_count, *value);
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                } else {
                    if(datatable_context->buffers[index]->float_samples[s]->value < tmp_value) {
                        tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                } else {
                    if(datatable_context->buffers[index]->float_samples[s]->value > tmp_value) {
                        tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                    tmp_ts = datatable_context->buffers[index]->float_samples[s]->value_ts;
                } else {
                    if(datatable_context->buffers[index]->float_samples[s]->value < tmp_value) {
                        tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                        tmp_ts = datatable_context->buffers[index]->float_samples[s]->value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                    tmp_ts = datatable_context->buffers[index]->float_samples[s]->value_ts;
                } else {
                    if(datatable_context->buffers[index]->float_samples[s]->value > tmp_value) {
                        tmp_value = datatable_context->buffers[index]->float_samples[s]->value;
                        tmp_ts = datatable_context->buffers[index]->float_samples[s]->value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
    }
 
    return ESP_OK;
}

/**
 * @brief Processes data-table int16 data-type data buffer samples on the stack by column based on the column index provided.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_int16_data_buffer(datatable_context_t *const datatable_context, const uint8_t index, int16_t *value, time_t *value_ts) {
    int16_t tmp_value = 0;
    time_t  tmp_ts    = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range for process int16 data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_INT16, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process int16 data buffer failed");

    // validate number of appended samples against expected number of samples
    if(datatable_context->processes[index]->samples_count != datatable_context->processes[index]->samples_size) {
        /* set default data */
        *value    = tmp_value;
        *value_ts = tmp_ts;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_context->processes[index]->process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value = datatable_context->buffers[index]->int16_samples[0]->value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                } else {
                    tmp_value += datatable_context->buffers[index]->int16_samples[s]->value;
                }
            }
            *value = tmp_value / datatable_context->processes[index]->samples_count;
            *value_ts = tmp_ts;
            ESP_LOGW(TAG, "datatable_process_int16_data_buffer(column-index: %u) data-count: %u data-avg: %u", index, datatable_context->processes[index]->samples_count, *value);
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                } else {
                    if(datatable_context->buffers[index]->int16_samples[s]->value < tmp_value) {
                        tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                } else {
                    if(datatable_context->buffers[index]->int16_samples[s]->value > tmp_value) {
                        tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                    tmp_ts = datatable_context->buffers[index]->int16_samples[s]->value_ts;
                } else {
                    if(datatable_context->buffers[index]->int16_samples[s]->value < tmp_value) {
                        tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                        tmp_ts = datatable_context->buffers[index]->int16_samples[s]->value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_context->processes[index]->samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                    tmp_ts = datatable_context->buffers[index]->int16_samples[s]->value_ts;
                } else {
                    if(datatable_context->buffers[index]->int16_samples[s]->value > tmp_value) {
                        tmp_value = datatable_context->buffers[index]->int16_samples[s]->value;
                        tmp_ts = datatable_context->buffers[index]->int16_samples[s]->value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
    }
 
    return ESP_OK;
}

esp_err_t datatable_init(const datatable_config_t *datatable_config, datatable_handle_t *datatable_handle) {
    esp_err_t ret = ESP_OK;

    /**
     * validate data-table sampling and processing type, period and offset arguments
     * 
     * the sampling rate must be lower than the processing interval. as an example,
     * a 5-sec sampling rate with a 1-min processing interval would trigger processing
     * of the row columns once every minute and would process 12 samples based on 
     * the desired processing type (i.e. avg, min, max).  if the processing type is
     * configured to smp a sample would be updated every sampling interval and latest
     * value would be recorded during the processing interval.
     */

    /* validate data-table arguments */
    ESP_GOTO_ON_FALSE( (strlen(datatable_config->name) <= DATATABLE_NAME_MAX_SIZE), ESP_ERR_INVALID_ARG, err, TAG, "data-table name cannot exceed 15-characters, data-table handle initialization failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->columns_size > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table columns size cannot be 0, data-table handle initialization failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->rows_size > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table rows size cannot be 0, data-table handle initialization failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->sampling_config.interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table sampling interval period cannot be 0, data-table handle initialization failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->processing_config.interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period cannot be 0, data-table handle initialization failed" );

    /* validate sampling and processing interval periods */
    int64_t interval_delta = time_into_interval_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_period) - 
                             time_into_interval_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_period); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the sampling interval period,  data-table handle initialization failed" );

    /* validate sampling period and offset intervals */
    interval_delta = time_into_interval_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_period) - 
                     time_into_interval_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_offset); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the sampling interval offset, data-table handle initialization failed" );
    
    /* validate processing period and offset intervals */
    interval_delta = time_into_interval_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_period) - 
                     time_into_interval_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_offset); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the processing interval offset, data-table handle initialization failed" );

    /* validate memory availability for data-table handle */
    datatable_context_t* datatable_context = (datatable_context_t*)calloc(1, sizeof(datatable_context_t));
    ESP_GOTO_ON_FALSE( datatable_context, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table handle, data-table handle initialization failed" );

    /* initialize data-table state object */
    datatable_context->name                 = datatable_config->name;
    datatable_context->columns_count        = 0;
    datatable_context->columns_size         = datatable_config->columns_size + 2; // add record id and timestamp columns
    datatable_context->rows_count           = 0;
    datatable_context->rows_size            = datatable_config->rows_size;
    datatable_context->sampling_count       = 0;
    datatable_context->data_storage_type    = datatable_config->data_storage_type;
    datatable_context->record_id            = 0;
    datatable_context->event_handler        = datatable_config->event_handler;
    datatable_context->hash_code            = datatable_get_hash_code();
    datatable_context->mutex_handle         = xSemaphoreCreateMutex();

    /* validate data-table mutex handle */
    ESP_GOTO_ON_FALSE( datatable_context->mutex_handle, ESP_ERR_INVALID_STATE, err_out_handle, TAG, "unable to create mutex, data-table handle initialization failed" );

    /* define default record id data-table column */
    datatable_column_t* dt_id_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_id_column, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table id column, data-table handle initialization failed" );
    dt_id_column->names[0].name      = DATATABLE_COLUMN_ID_NAME;
    dt_id_column->data_type          = DATATABLE_COLUMN_DATA_ID;
    datatable_context->columns_count       += 1;

    /* define default record id data-table process for column */
    datatable_process_t* dt_id_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_id_process, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table id process for column, data-table handle initialization failed" );
    dt_id_process->process_type      = DATATABLE_COLUMN_PROCESS_SMP;
    dt_id_process->samples_size      = 0;
    dt_id_process->samples_count     = 0;

    /* define default record timestamp data-table column */
    datatable_column_t* dt_ts_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_ts_column, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table timestamp column, data-table handle initialization failed" );
    dt_ts_column->names[0].name      = DATATABLE_COLUMN_TS_NAME;
    dt_ts_column->data_type          = DATATABLE_COLUMN_DATA_TS;
    datatable_context->columns_count       += 1;

    /* define default record timestamp data-table process for column */
    datatable_process_t* dt_ts_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_ts_process, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table timestamp process for column, data-table handle initialization failed" );
    dt_ts_process->process_type       = DATATABLE_COLUMN_PROCESS_SMP;
    dt_ts_process->samples_size       = 0;
    dt_ts_process->samples_count      = 0;

    // initialize task-schedule configuration - data-table sampling task system clock synchronization
    const time_into_interval_config_t dt_sampling_tii_cfg = {
        .name               = datatable_concat(datatable_config->name, DATATABLE_COLUMN_TII_SMP_NAME),
        .interval_type      = datatable_config->sampling_config.interval_type,
        .interval_period    = datatable_config->sampling_config.interval_period,
        .interval_offset    = datatable_config->sampling_config.interval_offset
    };

    // initialize a time-into-interval handle - data-table sampling task system clock synchronization
    ESP_GOTO_ON_ERROR( time_into_interval_init(&dt_sampling_tii_cfg, &datatable_context->sampling_tii_handle), 
                        err_out_handle, TAG, "unable to initialize sampling time-into-interval handle, data-table handle initialization failed" );

    // initialize time-into-interval configuration - data-table column data buffer processing task system clock synchronization
    const time_into_interval_config_t dt_processing_tii_cfg = {
        .name               = datatable_concat(datatable_config->name, DATATABLE_COLUMN_TII_PRC_NAME),
        .interval_type      = datatable_config->processing_config.interval_type,
        .interval_period    = datatable_config->processing_config.interval_period,
        .interval_offset    = datatable_config->processing_config.interval_offset
    };

    // initialize a time-into-interval handle - data-table column data buffer processing task system clock synchronization
    ESP_GOTO_ON_ERROR( time_into_interval_init(&dt_processing_tii_cfg, &datatable_context->processing_tii_handle), 
                        err_out_handle, TAG, "unable to initialize processing time-into-interval handle, data-table handle initialization failed" );

    /* set maximum size of data-table column data buffer */
    ESP_GOTO_ON_ERROR( datatable_get_column_samples_maximum_size(datatable_context, &datatable_context->samples_maximum_size),
                        err_out_handle, TAG, "unable to get column sample size maximum, data-table handle initialization failed" );

    /* validate memory availability for default data-table columns */
    datatable_context->columns = (datatable_column_t**)calloc(datatable_context->columns_size, sizeof(datatable_column_t*));
    ESP_GOTO_ON_FALSE( datatable_context->columns, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table columns, data-table handle initialization failed" );

    /* set all columns to null */
    for(uint8_t i = 0; i < datatable_context->columns_size; i++) {
        datatable_context->columns[i] = NULL;
    }

    /* append default data-table columns (record id[0] and timestamp[1]) to state object */
    datatable_context->columns[0] = dt_id_column;
    datatable_context->columns[1] = dt_ts_column;

    /* validate memory availability for default data-table column processes */
    datatable_context->processes = (datatable_process_t**)calloc(datatable_context->columns_size, sizeof(datatable_process_t*));
    ESP_GOTO_ON_FALSE( datatable_context->processes, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table column processes, data-table handle initialization failed" );

    /* set all column samples to null */
    for(uint8_t i = 0; i < datatable_context->columns_size; i++) {
        datatable_context->processes[i] = NULL;
    }

    /* append default data-table column processes (record id[0] and timestamp[1]) to state object */
    datatable_context->processes[0] = dt_id_process;
    datatable_context->processes[1] = dt_ts_process;

    /* validate memory availability for default data-table column buffers */
    datatable_context->buffers = (datatable_buffer_t**)calloc(datatable_context->columns_size, sizeof(datatable_buffer_t*));
    ESP_GOTO_ON_FALSE( datatable_context->buffers, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table column buffers, data-table handle initialization failed" );

    /* set all column samples to null */
    for(uint8_t i = 0; i < datatable_context->columns_size; i++) {
        datatable_context->buffers[i] = NULL;
    }

    /* validate memory availability for default data-table rows */
    datatable_context->rows = (datatable_row_t**)calloc(datatable_context->rows_size, sizeof(datatable_row_t*));
    ESP_GOTO_ON_FALSE( datatable_context->rows, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table rows, data-table handle initialization failed" );

    /* set all rows to null */
    for(uint16_t i = 0; i < datatable_context->rows_size; i++) {
        datatable_context->rows[i] = NULL;
    }

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_INIT, "initialized successfully");
    }

    /* set output handle */
    *datatable_handle = (datatable_handle_t)datatable_context;

    return ESP_OK;

    err_out_handle:
        free(datatable_context);
    err:
        return ret;
}

/**
 * @brief Appends a vector based data-type column to the data-table.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_vector_column(datatable_context_t *const datatable_context, const char *name_uc, const char *name_vc, const datatable_column_process_types_t process_type, uint8_t *index) {
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate column name length */
    ESP_GOTO_ON_FALSE( (strlen(name_uc) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, err_arg, TAG, "column name_uc is too long, data-table add vector column failed" );
    ESP_GOTO_ON_FALSE( (strlen(name_vc) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, err_arg, TAG, "column name_vc is too long, data-table add vector column failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate columns size */
    ESP_GOTO_ON_FALSE((datatable_context->columns_count + 1 <= datatable_context->columns_size), ESP_ERR_INVALID_SIZE, err_arg, TAG, "unable to add columns to data-table for add vector column");

    /* data-table column data buffer size of samples to process */
    uint16_t dt_samples_maximum_size = datatable_context->samples_maximum_size;

    /* validate data-table column data buffer size if process-type is a sample */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size (1-sample) for data-table column data buffer */
        dt_samples_maximum_size = 1;
    }

    /* validate memory availability for data-table column */
    datatable_column_t* dt_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_column, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table vector column, data-table handle add vector column failed" );

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name_uc, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name_vc, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_VECTOR;
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name_uc, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name_vc, process_type);
        dt_column->names[2].name = datatable_concat_column_name(name_vc, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_VECTOR;
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_GOTO_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, err_dt_column, TAG, "data-table column process-type is not supported float data-type, data-table add float column failed");
    }

    /* increment data-table columns count */
    datatable_context->columns_count += 1;

    /* set data-table column */
    datatable_context->columns[datatable_context->columns_count - 1] = dt_column;

    /* validate memory availability for data-table column process */
    datatable_process_t* dt_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_process, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table process for column, data-table handle initialization failed" );
    dt_process->process_type    = process_type;
    dt_process->samples_size    = dt_samples_maximum_size;
    dt_process->samples_count   = 0;

    /* set data-table process */
    datatable_context->processes[datatable_context->columns_count - 1] = dt_process;

    /* validate memory availability for data-table column buffer */
    datatable_buffer_t* dt_buffer = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
    ESP_GOTO_ON_FALSE( dt_buffer, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table buffer for column, data-table handle initialization failed" );

    /* validate memory availability for data-table column buffer samples */
    dt_buffer->vector_samples = (datatable_vector_column_data_type_t**)calloc(dt_samples_maximum_size, sizeof(datatable_vector_column_data_type_t*));
    ESP_GOTO_ON_FALSE(dt_buffer->vector_samples, ESP_ERR_NO_MEM, err_dt_samples, TAG, "no memory for data-table column buffer samples for add vector column");

    /* set all column buffer samples to null */
    for(uint8_t i = 0; i < dt_samples_maximum_size; i++) {
        dt_buffer->vector_samples[i] = NULL;
    }

    /* set data-table buffer */
    datatable_context->buffers[datatable_context->columns_count - 1] = dt_buffer;

    /* set output parameter */
    *index = datatable_context->columns_count - 1;

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;

    err_dt_samples:
        free(dt_buffer->vector_samples);
    err_dt_column:
        free(dt_column);
    err:
        xSemaphoreGive(datatable_context->mutex_handle);
    err_arg:
        return ret;
}

esp_err_t datatable_add_vector_smp_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add vector column for add vector sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_avg_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add vector column for add vector average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_min_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add vector column for add vector minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_max_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add vector column for add vector maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_min_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add vector column for add vector minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_max_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_context, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add vector column for add vector maximum with timestamp process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a bool based data-type column to the data-table.  This column data-type supports sampling only.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_bool_column(datatable_context_t *const datatable_context, const char *name, uint8_t *index) {
    esp_err_t   ret                 = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate column name length */
    ESP_GOTO_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, err_arg, TAG, "column name is too long, data-table add bool column failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate columns size */
    ESP_GOTO_ON_FALSE( (datatable_context->columns_count + 1 <= datatable_context->columns_size), ESP_ERR_INVALID_SIZE, err_arg, TAG, "unable to add columns to data-table for add bool column" );

    /* static size (1-sample) for data-table column data buffer with bool data-type */
    uint16_t dt_samples_maximum_size = 1;

    /* validate memory availability for data-table column */
    datatable_column_t* dt_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_column, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table bool column, data-table handle add bool column failed" );

    /* increment data-table columns count */
    datatable_context->columns_count += 1;
    
    /* initialize data-table column */
    dt_column->names[0].name    = datatable_concat_column_name(name, DATATABLE_COLUMN_PROCESS_SMP);
    dt_column->data_type        = DATATABLE_COLUMN_DATA_BOOL;

    /* set data-table column */
    datatable_context->columns[datatable_context->columns_count - 1] = dt_column;

    /* validate memory availability for data-table column process */
    datatable_process_t* dt_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_process, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table process for column, data-table handle initialization failed" );
    dt_process->process_type    = DATATABLE_COLUMN_PROCESS_SMP;
    dt_process->samples_size    = dt_samples_maximum_size;
    dt_process->samples_count   = 0;

    /* set data-table process */
    datatable_context->processes[datatable_context->columns_count - 1] = dt_process;

    /* validate memory availability for data-table column buffer */
    datatable_buffer_t* dt_buffer = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
    ESP_GOTO_ON_FALSE( dt_buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table buffer for column, data-table handle initialization failed" );

    /* validate memory availability for data-table column buffer samples */
    dt_buffer->bool_samples = (datatable_bool_column_data_type_t**)calloc(dt_samples_maximum_size, sizeof(datatable_bool_column_data_type_t*));
    ESP_GOTO_ON_FALSE(dt_buffer->bool_samples, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table column buffer samples for add bool column");

    /* set all column buffer samples to null */
    for(uint8_t i = 0; i < dt_samples_maximum_size; i++) {
        dt_buffer->bool_samples[i] = NULL;
    }

    /* set data-table buffer */
    datatable_context->buffers[datatable_context->columns_count - 1] = dt_buffer;

    /* set output parameter */
    *index = datatable_context->columns_count - 1;

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;

    err_dt_column:
        free(dt_column);
    err:
        xSemaphoreGive(datatable_context->mutex_handle);
    err_arg:
        return ret;
}

esp_err_t datatable_add_bool_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append bool sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_bool_column(datatable_context, name, index), TAG, "add bool column for add bool sample process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a float based data-type column to the data-table.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_float_column(datatable_context_t *const datatable_context, const char *name, const datatable_column_process_types_t process_type, uint8_t *index) {
    esp_err_t   ret              = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate column name length */
    ESP_GOTO_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, err_arg, TAG, "column name is too long, data-table add float column failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate columns size */
    ESP_GOTO_ON_FALSE( (datatable_context->columns_count + 1 <= datatable_context->columns_size), ESP_ERR_INVALID_SIZE, err_arg, TAG, "unable to add columns to data-table for add float column" );

    /* data-table column data buffer size of samples to process */
    uint16_t dt_samples_maximum_size = datatable_context->samples_maximum_size;

    /* validate data-table column data buffer size if process-type is a sample */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size (1-sample) for data-table column data buffer */
        dt_samples_maximum_size = 1;
    }

    /* validate memory availability for data-table column */
    datatable_column_t* dt_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_column, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table float column, data-table handle add float column failed" );

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column name */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_FLOAT;
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_FLOAT;
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_FLOAT;
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_GOTO_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, err_dt_column, TAG, "data-table column process-type is not supported float data-type, data-table add float column failed");
    }

    /* increment data-table columns count */
    datatable_context->columns_count += 1;

    /* set data-table column */
    datatable_context->columns[datatable_context->columns_count - 1] = dt_column;

    /* validate memory availability for data-table column process */
    datatable_process_t* dt_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_process, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table process for column, data-table handle initialization failed" );
    dt_process->process_type    = process_type;
    dt_process->samples_size    = dt_samples_maximum_size;
    dt_process->samples_count   = 0;

    /* set data-table process */
    datatable_context->processes[datatable_context->columns_count - 1] = dt_process;

    /* validate memory availability for data-table column buffer */
    datatable_buffer_t* dt_buffer = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
    ESP_GOTO_ON_FALSE( dt_buffer, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table buffer for column, data-table handle initialization failed" );

    /* validate memory availability for data-table column buffer samples */
    dt_buffer->float_samples = (datatable_float_column_data_type_t**)calloc(dt_samples_maximum_size, sizeof(datatable_float_column_data_type_t*));
    ESP_GOTO_ON_FALSE(dt_buffer->float_samples, ESP_ERR_NO_MEM, err_dt_samples, TAG, "no memory for data-table column buffer samples for add float column");

    /* set all column buffer samples to null */
    for(uint8_t i = 0; i < dt_samples_maximum_size; i++) {
        dt_buffer->float_samples[i] = NULL;
    }

    /* set data-table buffer */
    datatable_context->buffers[datatable_context->columns_count - 1] = dt_buffer;

    /* set output parameter */
    *index = datatable_context->columns_count - 1;

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;

    err_dt_samples:
        free(dt_buffer->float_samples);
    err_dt_column:
        free(dt_column);
    err:
        xSemaphoreGive(datatable_context->mutex_handle);
    err_arg:
        return ret;
}

esp_err_t datatable_add_float_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add float column for add float sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add float column for add float average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float minimum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add float column for add float minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float maximum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add float column for add float maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float minimum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add float column for add float minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append float maximum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add float column for add float maximum with timestamp process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a int16 based data-type column to the data-table.
 * 
 * @param[in] datatable_context Data-table context descriptor.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_int16_column(datatable_context_t *const datatable_context, const char *name, const datatable_column_process_types_t process_type, uint8_t *index) {
    esp_err_t   ret              = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate column name length */
    ESP_GOTO_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, err_arg, TAG, "column name is too long, data-table add int16 column failed" );

    /* lock the mutex */
    xSemaphoreTake(datatable_context->mutex_handle, portMAX_DELAY);

    /* validate columns size */
    ESP_GOTO_ON_FALSE( (datatable_context->columns_count + 1 <= datatable_context->columns_size), ESP_ERR_INVALID_SIZE, err_arg, TAG, "unable to add columns to data-table for add int16 column" );

    /* data-table column data buffer size of samples to process */
    uint16_t dt_samples_maximum_size = datatable_context->samples_maximum_size;

    /* validate data-table column data buffer size if process-type is a sample */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size (1-sample) for data-table column data buffer */
        dt_samples_maximum_size = 1;
    } 

    /* validate memory availability for data-table column */
    datatable_column_t* dt_column = (datatable_column_t*)calloc(1, sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( dt_column, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table int16 column, data-table handle add int16 column failed" );

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column name */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_INT16;
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_INT16;
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column names */
        dt_column->names[0].name = datatable_concat_column_name(name, process_type);
        dt_column->names[1].name = datatable_concat_column_name(name, process_type);
        dt_column->data_type     = DATATABLE_COLUMN_DATA_INT16;
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_GOTO_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, err_dt_column, TAG, "data-table column process-type is not supported int16 data-type, data-table add int16 column failed");
    }

    /* increment data-table columns count */
    datatable_context->columns_count += 1;

    /* set data-table column */
    datatable_context->columns[datatable_context->columns_count - 1] = dt_column;

    /* validate memory availability for data-table column process */
    datatable_process_t* dt_process = (datatable_process_t*)calloc(1, sizeof(datatable_process_t));
    ESP_GOTO_ON_FALSE( dt_process, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table process for column, data-table handle initialization failed" );
    dt_process->process_type    = process_type;
    dt_process->samples_size    = dt_samples_maximum_size;
    dt_process->samples_count   = 0;

    /* set data-table process */
    datatable_context->processes[datatable_context->columns_count - 1] = dt_process;

    /* validate memory availability for data-table column buffer */
    datatable_buffer_t* dt_buffer = (datatable_buffer_t*)calloc(1, sizeof(datatable_buffer_t));
    ESP_GOTO_ON_FALSE( dt_buffer, ESP_ERR_NO_MEM, err_dt_column, TAG, "no memory for data-table buffer for column, data-table handle initialization failed" );

    /* validate memory availability for data-table column buffer samples */
    dt_buffer->int16_samples = (datatable_int16_column_data_type_t**)calloc(dt_samples_maximum_size, sizeof(datatable_int16_column_data_type_t*));
    ESP_GOTO_ON_FALSE(dt_buffer->int16_samples, ESP_ERR_NO_MEM, err_dt_samples, TAG, "no memory for data-table column buffer samples for add int16 column");

    /* set all column buffer samples to null */
    for(uint8_t i = 0; i < dt_samples_maximum_size; i++) {
        dt_buffer->int16_samples[i] = NULL;
    }

    /* set data-table buffer */
    datatable_context->buffers[datatable_context->columns_count - 1] = dt_buffer;


    /* set output parameter */
    *index = datatable_context->columns_count - 1;

    /* unlock the mutex */
    xSemaphoreGive(datatable_context->mutex_handle);

    return ESP_OK;

    err_dt_samples:
        free(dt_buffer->float_samples);
    err_dt_column:
        free(dt_column);
    err:
        xSemaphoreGive(datatable_context->mutex_handle);
    err_arg:
        return ret;
}

esp_err_t datatable_add_int16_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add int16 column for add int16 sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add int16 column for add int16 average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 minimum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add int16 column for add int16 minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 maximum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add int16 column for add int16 maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 minimum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add int16 column for add int16 minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* append int16 maximum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_context, name, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add int16 column for add int16 maximum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_get_columns_count(datatable_handle_t datatable_handle, uint8_t *count) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* set output parameter */
    *count = datatable_context->columns_count;

    return ESP_OK;
}

esp_err_t datatable_get_rows_count(datatable_handle_t datatable_handle, uint8_t *count) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* set output parameter */
    *count = datatable_context->rows_count;

    return ESP_OK;
}

esp_err_t datatable_get_column(datatable_handle_t datatable_handle, const uint8_t index, datatable_column_t** column) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate index */
    ESP_RETURN_ON_FALSE( (index < datatable_context->columns_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, data-table handle get column failed" );

    /* set output parameter */
    *column = datatable_context->columns[index];

    return ESP_OK;
}

esp_err_t datatable_get_row(datatable_handle_t datatable_handle, const uint8_t index, datatable_row_t** row) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* validate index */
    ESP_RETURN_ON_FALSE( (index < datatable_context->rows_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, data-table handle get row failed" );

    /* set output parameter */
    *row = datatable_context->rows[index];

    return ESP_OK;
}

esp_err_t datatable_push_vector_sample(datatable_handle_t datatable_handle, const uint8_t index, const float value_uc, const float value_vc) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, push vector sample failed" );

    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_VECTOR, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push vector sample failed" );

    datatable_vector_column_data_type_t* dt_column_data = (datatable_vector_column_data_type_t*)calloc(1, sizeof(datatable_vector_column_data_type_t));
    ESP_RETURN_ON_FALSE( dt_column_data, ESP_ERR_NO_MEM, TAG, "no memory for data-table vector column data, push vector sample failed" );

    /* handle column process-type */
    if(datatable_context->processes[index]->process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_context->processes[index]->samples_count = 1;
        dt_column_data->value_ts = time_into_interval_get_epoch_timestamp();
        dt_column_data->value_uc = value_uc;
        dt_column_data->value_vc = value_vc;
    } else {
        // validate data buffer samples count
        if(datatable_context->processes[index]->samples_count + 1 > datatable_context->processes[index]->samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_context, index), TAG, "unable to fifo column data buffer, push vector sample failed" );

            // samples count remains the same but append sample to column data buffer
            dt_column_data->value_ts = time_into_interval_get_epoch_timestamp();
            dt_column_data->value_uc = value_uc;
            dt_column_data->value_vc = value_vc;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_context->processes[index]->samples_count += 1;
            dt_column_data->value_ts = time_into_interval_get_epoch_timestamp();
            dt_column_data->value_uc = value_uc;
            dt_column_data->value_vc = value_vc;
        }
    }

    datatable_context->buffers[index]->vector_samples[datatable_context->processes[index]->samples_count-1] = dt_column_data;

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_SAMPLE_PUSHED, "vector sample push onto the buffer samples stack successful");
    }

    return ESP_OK;
}

esp_err_t datatable_push_bool_sample(datatable_handle_t datatable_handle, const uint8_t index, const bool value) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, push bool sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_BOOL, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push bool sample failed" );

    /* validate column process-type */
    ESP_RETURN_ON_FALSE( datatable_context->processes[index]->process_type == DATATABLE_COLUMN_PROCESS_SMP, ESP_ERR_INVALID_ARG, TAG, "column process-type is incorrect, push bool sample failed" );

    datatable_bool_column_data_type_t* dt_column_data = (datatable_bool_column_data_type_t*)calloc(1, sizeof(datatable_bool_column_data_type_t));
    ESP_RETURN_ON_FALSE( dt_column_data, ESP_ERR_NO_MEM, TAG, "no memory for data-table bool column data, push bool sample failed" );

    /* handle column process-type */
    datatable_context->processes[index]->samples_count = 1;

    dt_column_data->value = value;

    datatable_context->buffers[index]->bool_samples[datatable_context->processes[index]->samples_count-1] = dt_column_data;

    ESP_LOGW(TAG, "datatable_push_bool_sample(column-index: %u) buffer-data(%d) %d", index, datatable_context->processes[index]->samples_count, datatable_context->buffers[index]->bool_samples[datatable_context->processes[index]->samples_count-1]->value);

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_SAMPLE_PUSHED, "bool sample push onto the buffer samples stack successful");
    }

    return ESP_OK;
}

esp_err_t datatable_push_float_sample(datatable_handle_t datatable_handle, const uint8_t index, const float value) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, push float sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_FLOAT, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push float sample failed");

    datatable_float_column_data_type_t* dt_column_data = (datatable_float_column_data_type_t*)calloc(1, sizeof(datatable_float_column_data_type_t));
    ESP_RETURN_ON_FALSE( dt_column_data, ESP_ERR_NO_MEM, TAG, "no memory for data-table float column data, push float sample failed" );
    
    /* handle column process-type */
    if(datatable_context->processes[index]->process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_context->processes[index]->samples_count = 1;
        dt_column_data->value_ts = time_into_interval_get_epoch_timestamp();
        dt_column_data->value    = value;
    } else {
        // validate data buffer samples index
        if(datatable_context->processes[index]->samples_count + 1 > datatable_context->processes[index]->samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_context, index), TAG, "unable to fifo column data buffer, push float sample failed" );

            // samples count remains the same and append sample to column data buffer
            dt_column_data->value_ts  = time_into_interval_get_epoch_timestamp();
            dt_column_data->value     = value;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_context->processes[index]->samples_count += 1;
            dt_column_data->value_ts  = time_into_interval_get_epoch_timestamp();
            dt_column_data->value     = value;
        }
    }

    datatable_context->buffers[index]->float_samples[datatable_context->processes[index]->samples_count-1] = dt_column_data;

    ESP_LOGW(TAG, "datatable_push_float_sample(column-index: %u) buffer-data(%d) %.2f", index, datatable_context->processes[index]->samples_count, datatable_context->buffers[index]->float_samples[datatable_context->processes[index]->samples_count-1]->value);

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_SAMPLE_PUSHED, "float sample push onto the buffer samples stack was successful");
    }

    return ESP_OK;
}

esp_err_t datatable_push_int16_sample(datatable_handle_t datatable_handle, const uint8_t index, const int16_t value) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_context, index), TAG, "column does not exist or index is out of range, push int16 sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_context->columns[index]->data_type == DATATABLE_COLUMN_DATA_INT16, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push int16 sample failed");

    datatable_int16_column_data_type_t* dt_column_data = (datatable_int16_column_data_type_t*)calloc(1, sizeof(datatable_int16_column_data_type_t));
    ESP_RETURN_ON_FALSE( dt_column_data, ESP_ERR_NO_MEM, TAG, "no memory for data-table int16 column data, push int16 sample failed" );

    /* handle column process-type */
    if(datatable_context->processes[index]->process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_context->processes[index]->samples_count = 1;
        dt_column_data->value_ts = time_into_interval_get_epoch_timestamp();
        dt_column_data->value    = value;
    } else {
        // validate data buffer samples index
        if(datatable_context->processes[index]->samples_count + 1 > datatable_context->processes[index]->samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_context, index), TAG, "unable to fifo column data buffer, push int16 sample failed" );

            // samples count remains the same but append sample to column data buffer
            dt_column_data->value_ts  = time_into_interval_get_epoch_timestamp();
            dt_column_data->value     = value;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_context->processes[index]->samples_count += 1;
            dt_column_data->value_ts  = time_into_interval_get_epoch_timestamp();
            dt_column_data->value     = value;
        }
    }

    datatable_context->buffers[index]->int16_samples[datatable_context->processes[index]->samples_count-1] = dt_column_data;

    ESP_LOGW(TAG, "datatable_push_int16_sample(column-index: %u) buffer-data(%d) %u", index, datatable_context->processes[index]->samples_count, datatable_context->buffers[index]->int16_samples[datatable_context->processes[index]->samples_count-1]->value);

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_SAMPLE_PUSHED, "int16 sample push onto the buffer samples stack successful");
    }

    return ESP_OK;
}

esp_err_t datatable_sampling_task_delay(datatable_handle_t datatable_handle) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    /* delay data-table sampling task per sampling task-schedule handle */
    ESP_RETURN_ON_ERROR( time_into_interval_delay(datatable_context->sampling_tii_handle), TAG, "unable to delay time-into-interval, data-table sampling time-into-interval delay failed" );
    
    return ESP_OK;
}

esp_err_t datatable_process_samples(datatable_handle_t datatable_handle) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    // increment sampling count, TODO - see if this can be sync'd to the sampling task
    datatable_context->sampling_count += 1;

    /* validate data-table processing interval event */
    if(time_into_interval(datatable_context->processing_tii_handle) == false) {
        /* data-table processing time-into-interval hasn't elapsed, exit the function */
        return ESP_OK;
    }

    /* invoke event handler */
    if(datatable_context->event_handler) {
        datatable_invoke_event(datatable_context, DATATABLE_EVENT_PROCESS_ELAPSED, "process samples time-into-interval has elapsed");
    }

    /* validate data buffer samples processing condition */
    if(datatable_context->sampling_count >= datatable_context->samples_maximum_size) {
        /* reset sampling count */
        datatable_context->sampling_count = 0;
    } else if(datatable_context->sampling_count < datatable_context->samples_maximum_size) {
        /* insufficient number of data buffer samples at processing interval event, reset column data buffers */
        ESP_RETURN_ON_ERROR( datatable_reset_data_buffers(datatable_context), TAG, "reset column data buffer for data-table process samples failed" );

        /* exit the function */
        return ESP_OK;
    } else { 
        /* reset sampling count */
        datatable_context->sampling_count = 0;

        /* unsupported condition */
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* handle data-table row count and index, and storage */
    if(datatable_context->rows_count == 0) {
        /* initialize data-table row count  */
        datatable_context->rows_count = 1;
    } else {
        /* increment data-table row count  */
        datatable_context->rows_count += 1;

        /* if the data-table is full, decrement row count  */
        if(datatable_context->rows_count > datatable_context->rows_size) {
            datatable_context->rows_count -= 1;

            ESP_LOGE(TAG, "datatable_process_samples rows_count %d", datatable_context->rows_count);

            switch(datatable_context->data_storage_type) {
                case DATATABLE_DATA_STORAGE_MEMORY_RING:
                    // pop first row and push remaining rows to top of stack
                    ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_context), TAG, "data-table fifo rows for process samples failed" );
                    break;
                case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                    // reset data-table by clearing rows
                    ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_context), TAG, "data-table reset rows for process samples failed" );
                    break;
                case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                    // stop processing samples by exiting function
                    return ESP_OK; // TODO log as debug info
                    break;
            }
        }
    }

    /* validate memory availability for data-table row */
    datatable_row_t* dt_row = (datatable_row_t*)calloc(1, sizeof(datatable_row_t));
    ESP_RETURN_ON_FALSE( dt_row, ESP_ERR_NO_MEM, TAG, "no memory for data-table row, data-table handle process samples failed" );

    /* validate memory availability for data-table row data columns */
    dt_row->data_columns = (datatable_row_data_column_t**)calloc(datatable_context->columns_size, sizeof(datatable_row_data_column_t*));
    ESP_RETURN_ON_FALSE( dt_row->data_columns, ESP_ERR_NO_MEM, TAG, "no memory for data-table row data columns, data-table handle process samples failed");

    /* process data-table row data columns by data-type for each column */
    for(uint8_t i = 0; i < datatable_context->columns_count; i++) {
        /* validate memory availability for data-table row data column */
        datatable_row_data_column_t* dt_data = (datatable_row_data_column_t*)calloc(1, sizeof(datatable_row_data_column_t));
        ESP_RETURN_ON_FALSE( dt_data, ESP_ERR_NO_MEM, TAG, "no memory for data-table row data column, data-table handle process samples failed" );

        // process data-table buffer data by data-type
        switch(datatable_context->columns[i]->data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                datatable_context->record_id++;
                dt_data->id_data.value = datatable_context->record_id;
                break;
            case DATATABLE_COLUMN_DATA_TS:
                dt_data->ts_data.value = time_into_interval_get_epoch_timestamp(); // unix epoch timestamp in seconds
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                ESP_RETURN_ON_ERROR( datatable_process_vector_data_buffer(datatable_context, i, 
                                                                        &dt_data->vector_data.value_uc, 
                                                                        &dt_data->vector_data.value_vc, 
                                                                        &dt_data->vector_data.value_ts), 
                                                                        TAG, "process vector data buffer for data-table process samples failed" );
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_context, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                ESP_RETURN_ON_ERROR( datatable_process_bool_data_buffer(datatable_context, i, 
                                                                        &dt_data->bool_data.value), 
                                                                        TAG, "process bool data buffer for data-table process samples failed" );
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_context, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                ESP_RETURN_ON_ERROR( datatable_process_float_data_buffer(datatable_context, i, 
                                                                        &dt_data->float_data.value, 
                                                                        &dt_data->float_data.value_ts), 
                                                                        TAG, "process float data buffer for data-table process samples failed" );
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_context, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                ESP_RETURN_ON_ERROR( datatable_process_int16_data_buffer(datatable_context, i, 
                                                                        &dt_data->int16_data.value, 
                                                                        &dt_data->int16_data.value_ts), 
                                                                        TAG, "process int16 data buffer for data-table process samples failed" );
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_context, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
        }

        /* set data-table row data column */
        dt_row->data_columns[i] = dt_data;
    }


    /* set data-table row */
    datatable_context->rows[datatable_context->rows_count - 1] = dt_row;

    /* invoke data-logger event */
    datatable_invoke_event(datatable_context, DATATABLE_EVENT_PROCESS, "samples processed successfully");

    return ESP_OK;
}

esp_err_t datatable_delete(datatable_handle_t datatable_handle) {
    /* free resource */
    if(datatable_handle) {
        // todo - free subentities
        free(datatable_handle);
    }

    return ESP_OK;
}

static inline esp_err_t datatable_create_json_columns(datatable_context_t *const datatable_context, cJSON **columns) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    // create columns array for data-table
    cJSON *json_columns = cJSON_CreateArray();

    // render each data-table column to json column object
    for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
        datatable_column_t*  dt_column  = datatable_context->columns[ci];
        datatable_process_t* dt_process = datatable_context->processes[ci];
        
        /* handle basic and complex data-types */
        if(dt_column->data_type == DATATABLE_COLUMN_DATA_ID || dt_column->data_type == DATATABLE_COLUMN_DATA_TS ||
           dt_column->data_type == DATATABLE_COLUMN_DATA_BOOL) {
            cJSON *json_column = cJSON_CreateString(dt_column->names[0].name);

            // set column attributes and append column to array
            cJSON_AddItemToArray(json_columns, json_column);
        } else {
            /* handle complex data-types*/
            if(dt_column->data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column_0 = cJSON_CreateString(dt_column->names[0].name); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(dt_column->names[1].name); // v-component

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
                } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(dt_column->names[0].name); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(dt_column->names[1].name); // v-component
                    cJSON *json_column_2 = cJSON_CreateString(dt_column->names[2].name); // timestamp

                    /* 3 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);

                    // set column 2 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_2);
                }
            } else if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column->data_type == DATATABLE_COLUMN_DATA_INT16) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column = cJSON_CreateString(dt_column->names[0].name);

                    // set column attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column);
               } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(dt_column->names[0].name);
                    cJSON *json_column_1 = cJSON_CreateString(dt_column->names[1].name);

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
               } /* process-types */   
            } /* complex data-types */
        } /* simple data-types */
    } /* for each data-table column */

    *columns = json_columns;

    return ESP_OK;
}

static inline esp_err_t datatable_create_json_types(datatable_context_t *const datatable_context, cJSON **types) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    // create columns array for data-table
    cJSON *json_columns = cJSON_CreateArray();

    // render each data-table column to json column object
    for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
        datatable_column_t*  dt_column  = datatable_context->columns[ci];
        datatable_process_t* dt_process = datatable_context->processes[ci];
        
        /* handle basic and complex data-types */
        if(dt_column->data_type == DATATABLE_COLUMN_DATA_ID || dt_column->data_type == DATATABLE_COLUMN_DATA_TS ||
           dt_column->data_type == DATATABLE_COLUMN_DATA_BOOL) {
            cJSON *json_column = cJSON_CreateString(datatable_json_serialize_column_data_type(dt_column->data_type));

            // set column attributes and append column to array
            cJSON_AddItemToArray(json_columns, json_column);
        } else {
            /* handle complex data-types*/
            if(dt_column->data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT)); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT)); // v-component

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
                } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT)); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT)); // v-component
                    cJSON *json_column_2 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS)); // timestamp

                    /* 3 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);

                    // set column 2 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_2);
                }
            } else if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column->data_type == DATATABLE_COLUMN_DATA_INT16) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column = cJSON_CreateString(datatable_json_serialize_column_data_type(dt_column->data_type));

                    // set column attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column);
               } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_column_data_type(dt_column->data_type));
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS));

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
               } /* process-types */   
            } /* complex data-types */
        } /* simple data-types */
    } /* for each data-table column */

    *types = json_columns;

    return ESP_OK;
}

static inline esp_err_t datatable_create_json_processes(datatable_context_t *const datatable_context, cJSON **processes) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_context );

    // create columns array for data-table
    cJSON *json_columns = cJSON_CreateArray();

    // render each data-table column to json column object
    for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
        datatable_column_t*  dt_column  = datatable_context->columns[ci];
        datatable_process_t* dt_process = datatable_context->processes[ci];
        
        /* handle basic and complex data-types */
        if(dt_column->data_type == DATATABLE_COLUMN_DATA_ID || dt_column->data_type == DATATABLE_COLUMN_DATA_TS ||
           dt_column->data_type == DATATABLE_COLUMN_DATA_BOOL) {
            cJSON *json_column = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type));

            // set column attributes and append column to array
            cJSON_AddItemToArray(json_columns, json_column);
        } else {
            /* handle complex data-types*/
            if(dt_column->data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type)); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type)); // v-component

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
                } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type)); // u-component
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type)); // v-component
                    cJSON *json_column_2 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type)); // timestamp

                    /* 3 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);

                    // set column 2 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_2);
                }
            } else if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column->data_type == DATATABLE_COLUMN_DATA_INT16) {
                /* handle process-types */
                if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type));

                    // set column attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column);
               } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type));
                    cJSON *json_column_1 = cJSON_CreateString(datatable_json_serialize_process_type(dt_process->process_type));

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddItemToArray(json_columns, json_column_1);
               } /* process-types */   
            } /* complex data-types */
        } /* simple data-types */
    } /* for each data-table column */

    *processes = json_columns;

    return ESP_OK;
}

esp_err_t datatable_to_json(datatable_handle_t datatable_handle, cJSON **datatable) {
    datatable_context_t* datatable_context = (datatable_context_t*)datatable_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    // create root object for data-table
    cJSON *json_table = cJSON_CreateObject();

    time_into_interval_types_t prc_interval_type;
    uint16_t prc_interval_period;

    // get processing interval type and period
    ESP_RETURN_ON_ERROR( time_into_interval_get_interval(datatable_context->processing_tii_handle, &prc_interval_type, &prc_interval_period), TAG, "unable to get data-table processing interval type and period" );

    // set data-table attributes
    cJSON_AddStringToObject(json_table, "name", datatable_context->name);
    cJSON_AddStringToObject(json_table, "process-interval", datatable_json_serialize_interval_type(prc_interval_type));
    cJSON_AddNumberToObject(json_table, "process-period", prc_interval_period);

    // create columns array for data-table
    cJSON *json_columns;
    datatable_create_json_columns(datatable_context, &json_columns);

    // append rendered columns to data-table
    cJSON_AddItemToObject(json_table, "columns", json_columns);

    // create types array for data-table
    cJSON *json_types;
    datatable_create_json_types(datatable_context, &json_types);

    // append rendered types to data-table
    cJSON_AddItemToObject(json_table, "types", json_types);

    // create processes array for data-table
    cJSON *json_processes;
    datatable_create_json_processes(datatable_context, &json_processes);

    // append rendered processes to data-table
    cJSON_AddItemToObject(json_table, "processes", json_processes);


    /* validate rows count */
    if(datatable_context->rows_count > 0) {
        // create rows array for data-table
        cJSON *json_rows = cJSON_CreateArray();

        // render each data-table row to json row object
        for(uint16_t ri = 0; ri < datatable_context->rows_count; ri++) {
            datatable_row_t* dt_row = datatable_context->rows[ri];

            // create row data columns array
            cJSON *json_row_data_columns = cJSON_CreateArray();

            if(dt_row == NULL || dt_row->data_columns == NULL) continue;

            // render each data-table row data column
            for(uint8_t ci = 0; ci < datatable_context->columns_count; ci++) {
                datatable_column_t*          dt_column          = datatable_context->columns[ci];
                datatable_process_t*         dt_process         = datatable_context->processes[ci];
                datatable_row_data_column_t* dt_row_data_column = dt_row->data_columns[ci];

                /* validate row-data-column instance */
                if(dt_row_data_column != NULL) {

                    /* handle basic and complex data-types */
                    if(dt_column->data_type == DATATABLE_COLUMN_DATA_ID || dt_column->data_type == DATATABLE_COLUMN_DATA_TS ||
                        dt_column->data_type == DATATABLE_COLUMN_DATA_BOOL) {
                        cJSON *json_row_data_column;

                        // set row data column attributes

                        /* handle data-type */
                        if(dt_column->data_type == DATATABLE_COLUMN_DATA_ID) {
                            json_row_data_column = cJSON_CreateNumber(dt_row_data_column->id_data.value);
                        } else if(dt_column->data_type == DATATABLE_COLUMN_DATA_TS) {
                            json_row_data_column = cJSON_CreateNumber(dt_row_data_column->ts_data.value);
                        } else {
                            json_row_data_column = cJSON_CreateNumber(dt_row_data_column->bool_data.value);
                        }

                        // append rendered row data column to row data columns array
                        cJSON_AddItemToArray(json_row_data_columns, json_row_data_column);
                    } else {
                        /* handle complex data-types*/
                        if(dt_column->data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                            /* handle process-types */
                            if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                                dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                                cJSON *json_row_data_column_0 = cJSON_CreateNumber(dt_row_data_column->vector_data.value_uc);  // u-component
                                cJSON *json_row_data_column_1 = cJSON_CreateNumber(dt_row_data_column->vector_data.value_vc);  // v-component

                                /* 2-columns */

                                // append rendered row data column 0 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                                // append rendered row data column 1 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);
                            } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                                cJSON *json_row_data_column_0 = cJSON_CreateNumber(dt_row_data_column->vector_data.value_uc);  // u-component
                                cJSON *json_row_data_column_1 = cJSON_CreateNumber(dt_row_data_column->vector_data.value_vc);  // v-component
                                cJSON *json_row_data_column_2 = cJSON_CreateNumber(dt_row_data_column->vector_data.value_ts);  // timestamp

                                /* 3-columns */

                                // append rendered row data column 0 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                                // append rendered row data column 1 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);

                                // append rendered row data column 0 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_2);
                            }
                        } else if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column->data_type == DATATABLE_COLUMN_DATA_INT16) {
                            /* handle process-types */
                            if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_process->process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                                dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                                cJSON *json_row_data_column;

                                // set row data column attributes

                                /* handle data-type */
                                if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                    json_row_data_column = cJSON_CreateNumber(dt_row_data_column->float_data.value);
                                } else {
                                    json_row_data_column = cJSON_CreateNumber(dt_row_data_column->int16_data.value);
                                }

                                // append rendered row data column to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column);
                            } else if(dt_process->process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_process->process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                                /* 2 columns: value with timestamp */
                                cJSON *json_row_data_column_0;
                                cJSON *json_row_data_column_1;

                                // set row data column 0 attributes - value

                                /* handle data-type for row data column 0 */
                                if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                    json_row_data_column_0 = cJSON_CreateNumber(dt_row_data_column->float_data.value);
                                } else {
                                    json_row_data_column_0 = cJSON_CreateNumber(dt_row_data_column->int16_data.value);
                                }

                                // append rendered row data column 0 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                                // set row data column 1 attributes - value

                                /* handle data-type for row data column 0 */
                                if(dt_column->data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                    json_row_data_column_1 = cJSON_CreateNumber(dt_row_data_column->float_data.value_ts);
                                } else {
                                    json_row_data_column_1 = cJSON_CreateNumber(dt_row_data_column->int16_data.value_ts);
                                }

                                // append rendered row data column 1 to row data columns array
                                cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);
                            } /* process-types */ 
                        } /* complex data-types */
                    } /* simple data-types */
                } /* row-data-column null check */
            } // for-each row-data-column

            // append rendered row data columns array to rows
            cJSON_AddItemToArray(json_rows, json_row_data_columns);
        } // for-each row

        // append rendered rows to data-table
        cJSON_AddItemToObject(json_table, "rows", json_rows);
    } /* row count */


    /* set json output table */
    *datatable = json_table;

    return ESP_OK;
}
