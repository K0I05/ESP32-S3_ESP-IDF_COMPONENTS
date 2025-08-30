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
 * @file datatable.h
 * @defgroup drivers datatable
 * @{
 *
 * ESP-IDF library for datatable
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DATATABLE_H__
#define __DATATABLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <cJSON.h>
#include <time_into_interval.h>
#include "datalogger_version.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * ESP DATA-TABLE definitions
 */
#define DATATABLE_NAME_MAX_SIZE         (15)        //!< 15-characters for user-defined table name
#define DATATABLE_COLUMN_NAME_SIZE      (15)        //!< 15-characters for user-defined column name 
#define DATATABLE_COLUMN_NAME_MAX_SIZE  (25)        //!< 25-characters for column name
#define DATATABLE_COLUMNS_MAX           (255)       //!< 
#define DATATABLE_ROWS_MAX              (65535)     //!< 
#define DATATABLE_COLUMN_ID_NAME        "Record ID"
#define DATATABLE_COLUMN_TS_NAME        "TS"
#define DATATABLE_COLUMN_TII_SMP_NAME   "_tii_smp"
#define DATATABLE_COLUMN_TII_PRC_NAME   "_tii_prc"

/*
 * ESP DATA-TABLE macro definitions
 */

/*
 * ESP DATA-TABLE enum and struct definitions
 */

/**
 * @brief Data-table event types enumerator.
 */
typedef enum datatable_event_types_tag {
    DATATABLE_EVENT_INIT,               /*!< data-table initialized successfully */
    DATATABLE_EVENT_RESET_ROWS,         /*!< data-table rows were reset */
    DATATABLE_EVENT_RESET_SAMPLES,      /*!< data-table buffer samples were reset */
    DATATABLE_EVENT_FIFO_ROWS,          /*!< data-table rows underwent a FIFO operation */
    DATATABLE_EVENT_FIFO_SAMPLES,       /*!< data-table buffer samples underwent a FIFO operation */
    DATATABLE_EVENT_SAMPLE_PUSHED,      /*!< data-table sample was pushed onto the buffer samples stack */
    DATATABLE_EVENT_PROCESS,            /*!< data-table processed successfully */
    DATATABLE_EVENT_PROCESS_ELAPSED,    /*!< data-table processing time-into-interval has elapsed */
} datatable_event_types_t;

/**
 * @brief Data-table data storage-types enumerator.
 */
typedef enum datatable_data_storage_types_tag {
    DATATABLE_DATA_STORAGE_MEMORY_RING,     /*!< data-table ring memory performs a first-in first-out (FIFO) with the data-table */
    DATATABLE_DATA_STORAGE_MEMORY_RESET,    /*!< data-table memory reset performs a data-table reset when data-table is full */
    DATATABLE_DATA_STORAGE_MEMORY_STOP      /*!< data-table memory stop pauses data storage when data-table is full */
} datatable_data_storage_types_t;

/**
 * @brief Data-table column statistical process-types enumerator.
 */
typedef enum datatable_column_process_types_tag {
    DATATABLE_COLUMN_PROCESS_SMP,       /*!< a sample is stored at every processing interval */
    DATATABLE_COLUMN_PROCESS_AVG,       /*!< stored samples are averaged over the processing interval */
    DATATABLE_COLUMN_PROCESS_MIN,       /*!< stored samples are analyzed for minimum over the processing interval */
    DATATABLE_COLUMN_PROCESS_MAX,       /*!< stored samples are analyzed for maximum over the processing interval */
    DATATABLE_COLUMN_PROCESS_MIN_TS,    /*!< stored samples are analyzed for minimum with timestamp over the processing interval */
    DATATABLE_COLUMN_PROCESS_MAX_TS,    /*!< stored samples are analyzed for maximum with timestamp over the processing interval */
} datatable_column_process_types_t;

/**
 * @brief Data-table column data-types enumerator.
 */
typedef enum datatable_column_data_types_tag {
    DATATABLE_COLUMN_DATA_ID,       /*!< record identifier column data-type, system default, see `datatable_id_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_TS,       /*!< record timestamp (date and time) column data type, system default, see `datatable_ts_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_VECTOR,   /*!< vector (u and v components) column data type, user-defined, see `datatable_vector_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_BOOL,     /*!< boolean column data type, user-defined, see `datatable_bool_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_FLOAT,    /*!< float 32-bit column data type, user-defined, see `datatable_float_data_type_t` for data-type structure. */
    //DATATABLE_COLUMN_DATA_FP16,     /*!< float 16-bit column data type, user-defined, see `datatable_fp16_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_INT16     /*!< int16 column data type, user-defined, see `datatable_int16_data_type_t` for data-type structure. */
} datatable_column_data_types_t;


/**
 * @brief Data-table event structure.
 * 
 */
typedef struct datatable_event_tag {
    datatable_event_types_t type;
    const char*             message;
} datatable_event_t;

/**
 * @brief Data-logger event.
 * 
 */
typedef void (*datatable_event)(void *context, datatable_event_t);

/**
 * @brief Data-table record identifier column data-type structure.
 */
typedef struct datatable_id_column_data_type_tag {
    uint16_t                            value;      // record id value   
} datatable_id_column_data_type_t;

/**
 * @brief Data-table record timestamp (utc) column data-type structure.
 */
typedef struct datatable_ts_column_data_type_tag {
    time_t                              value;      // timestamp value    
} datatable_ts_column_data_type_t;

/**
 * @brief Data-table vector data-type column structure.
 */
typedef struct datatable_vector_column_data_type_tag {
    float                               value_uc;   // u-component (angle) value 
    float                               value_vc;   // v-component (velocity) value
    time_t                              value_ts;   // timestamp of values, used for time of max or min
} datatable_vector_column_data_type_t;

/**
 * @brief Data-table bool data-type column structure.
 */
typedef struct datatable_bool_column_data_type_tag {
    bool                                value;      // boolean value 
} datatable_bool_column_data_type_t;

/**
 * @brief Data-table float data-type column structure.
 */
typedef struct datatable_float_column_data_type_tag {
    float                               value;      // float 32-bit value
    time_t                              value_ts;   // timestamp of value, used for time of max or min  
} datatable_float_column_data_type_t;

/**
 * @brief Data-table float 16-bit data-type column structure.
 */
typedef struct datatable_fp16_column_data_type_tag {
    uint16_t                            value;      // float 16-bit value (stored as a uint16_t)
    time_t                              value_ts;   // timestamp of value, used for time of max or min  
} datatable_fp16_column_data_type_t;

/**
 * @brief Data-table int16 data-type column structure.
 */
typedef struct datatable_int16_column_data_type_tag {
    int16_t                             value;      // int16 value
    time_t                              value_ts;   // timestamp of value, used for time of max or min   
} datatable_int16_column_data_type_t;

/**
 * @brief Data-table column name structure.
 */
typedef struct datatable_column_name_tag {
    const char*                         name;      // data-table column name, maximum 15 characters.
} datatable_column_name_t;

/**
 * @brief Data-table column structure.  The data-table record identifier and timestamp columns
 * are created by default when the data-table is created.  The record identifier and
 * record timestamp data-types are excluded from data processing.
 */
typedef struct datatable_column_tag {
    datatable_column_name_t             names[3];           // data-table column names, index 0 and 1 for vector data-type or index 0, 1, and 2 for max and min with timestamp process-types, and index 0 for all other scenarios.
    datatable_column_data_types_t       data_type;          // data-table column data-type, automatically populated when row is created.
} datatable_column_t;

/**
 * @brief Data-table process structure.
 */
typedef struct datatable_process_tag {
    uint16_t                            samples_size;       // data-table size of data buffer samples, automatically populated when column is created
    uint16_t                            samples_count;      // data-table number of samples in the data buffer, automatically populated when data-table is processed
    datatable_column_process_types_t    process_type;       // data-table statistical data processing type setting.
} datatable_process_t;

/**
 * @brief Data-table buffer union structure.
 */
typedef union datatable_buffer_tag {
    datatable_vector_column_data_type_t** vector_samples;     // data-table vector samples data buffer, automatic array sizing when column is created based configured column data-type
    datatable_bool_column_data_type_t**   bool_samples;       // data-table boolean samples data buffer, automatic array sizing when column is created based configured column data-type
    datatable_float_column_data_type_t**  float_samples;      // data-table float samples data buffer, automatic array sizing when column is created based configured column data-type
    datatable_int16_column_data_type_t**  int16_samples;      // data-table int16 samples data buffer, automatic array sizing when column is created based configured column data-type
} datatable_buffer_t;


/**
 * @brief Data-table row data column structure.  This structure is a data model that represents 
 * data storage of the record based on the data-table's column column data-type.
 */
typedef union datatable_row_data_column_tag {
     datatable_id_column_data_type_t        id_data;            // data-table column record identifier data-type structure, automatically populated when row is created.
     datatable_ts_column_data_type_t        ts_data;            // data-table column record timestamp data-type structure, automatically populated when row is created.
     datatable_vector_column_data_type_t    vector_data;        // data-table column unit-vector data-type structure, automatically populated when row is created.
     datatable_bool_column_data_type_t      bool_data;          // data-table column boolean data-type structure, automatically populated when row is created.
     datatable_float_column_data_type_t     float_data;         // data-table column float data-type structure, automatically populated when row is created.
     datatable_int16_column_data_type_t     int16_data;         // data-table column int16 data-type structure, automatically populated when row is created.
} datatable_row_data_column_t;

/**
 * @brief Data-table row structure.  This structure is a data model that represents
 * data storage of record by data-table row and configured data-table columns.
 */
typedef struct datatable_row_tag {
    datatable_row_data_column_t**   data_columns;       // data-table data for each column of the record contained in the row, automatically populated when row is created.
} datatable_row_t;


/**
 * @brief Data-table configuration structure definition.
 */
typedef struct datatable_config_tag {
    const char*                         name;                       /*!< data-table textual name, maximum of 15 characters */
    uint8_t                             columns_size;               /*!< data-table column array size, 1..255, this setting cannot be 0 */
    uint16_t                            rows_size;                  /*!< data-table row array size, 1..65535, this setting cannot be 0 */
    datatable_data_storage_types_t      data_storage_type;          /*!< data-table data storage type, defines handling of records when the data-table is full */
    time_into_interval_config_t         sampling_config;            /*!< data-table sampling time-into-interval configuration, configures the sampling interval  */
    time_into_interval_config_t         processing_config;          /*!< data-table processing time-into-interval configuration, configures the processing interval */
    datatable_event                     event_handler;
} datatable_config_t;


 /**
  * @brief Data-table opaque handle structure definition.
  */
typedef void* datatable_handle_t;



/*
 * ESP DATA-TABLE sub-routine and function definitions
 */



/**
 * @brief Initializes a data-table handle.  A data-table handle instance is required before any other
 * data-table functions can be called.  Once the data-table is initialized the following functions are
 * used to configure the data-table columns and within the sampling task.
 * 
 * Use the `datatable_add_[data-type]_[process-type]_column` functions to define data-table columns   
 * by data-type and process-type.  The data-table columns are ordered as they are added and column index 
 * for the first user-defined column always starts at 2 given  the record identifier and timestamp columns  
 * are created by default and consume column indexes 0 and 1 respectively.
 * 
 * Use the `datatable_sampling_task_delay` function within the sampling task to sample measurements 
 * at the data-table's configured sampling interval.
 * 
 * Use the `datatable_push_[data-type]_sample` functions within the sampling task to push samples 
 * onto the column data buffer stack for processing.  These functions should be placed after the
 * `datatable_sampling_task_delay` function.
 * 
 * Use the `datatable_process_samples` function within the sampling task to process column data buffer 
 * samples at the data-table's configured processing interval.  This function should be placed after 
 * the `datatable_push_[data-type]_sample` functions.
 * 
 * @param[in] datatable_config Data-table configuration.
 * @param[out] datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_init(const datatable_config_t *datatable_config, datatable_handle_t *datatable_handle);

/**
 * @brief Appends a vector based data-type column as a sample to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_smp_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as an average to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_avg_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component minimum to the data-table.
 * 
 * The u-component at v-component minimum is sampled and stored.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_min_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component maximum to the data-table.
 * 
 * The u-component at v-component maximum is sampled and stored.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_max_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component minimum with timestamp to the data-table.
 * 
 * The u-component at v-component minimum is sampled and stored.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_min_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component maximum with timestamp to the data-table.
 * 
 * The u-component at v-component maximum is sampled and stored.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_max_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index);

/**
 * @brief Appends a bool based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_bool_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as an average process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a minimum process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a maximum process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a minimum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a maximum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as an average process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a minimum process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a maximum process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a minimum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a maximum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index);

/**
 * @brief Gets the number of columns in the data-table.
 * 
 * @param datatable_handle Data-table handle.
 * @param count Number of columns in the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_columns_count(datatable_handle_t datatable_handle, uint8_t *count);

/**
 * @brief Gets the number of rows in the data-table.
 * 
 * @param datatable_handle Data-table handle.
 * @param count Number of rows in the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_rows_count(datatable_handle_t datatable_handle, uint8_t *count);

/**
 * @brief Gets the column structure from the data-table based on the column index.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Data-table column index to output.
 * @param column Data-table column structure output.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_column(datatable_handle_t datatable_handle, const uint8_t index, datatable_column_t** column);

/**
 * @brief Gets the row structure from the data-table based on the row index.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Data-table row index to output.
 * @param row Data-table row structure output.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_row(datatable_handle_t datatable_handle, const uint8_t index, datatable_row_t** row);

/**
 * @brief Pushes a vector data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Sample data-table column index.
 * @param uc_value Vector data-type u-component sample to process.
 * @param vc_value Vector data-type v-component sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_vector_sample(datatable_handle_t datatable_handle, const uint8_t index, const float uc_value, const float vc_value);

/**
 * @brief Pushes a boolean data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Sample data-table column index.
 * @param value Boolean data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_bool_sample(datatable_handle_t datatable_handle, const uint8_t index, const bool value);

/**
 * @brief Pushes a float data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Sample data-table column index..
 * @param value Float data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_float_sample(datatable_handle_t datatable_handle, const uint8_t index, const float value);

/**
 * @brief Pushes an int16 data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Sample data-table column index.
 * @param value Int16 data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_int16_sample(datatable_handle_t datatable_handle, const uint8_t index, const int16_t value);

/**
 * @brief Delays the data-table's sampling task until the next scheduled task event.  
 * This function should be placed after the `for (;;) {` syntax to delay the task based 
 * on the configured time-into-interval handle interval type,  period, and offset parameters.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_sampling_task_delay(datatable_handle_t datatable_handle);

/**
 * @brief Processes data-table samples on the data buffer stack in each column based on the data-table's  
 * configured processing interval. When the samples are processed, the data buffer stack is cleared 
 * for each column.  This function must be called after data-table samples are pushed in the sampling task.
 * 
 * If the sampling period exceeds the data-table's configured sampling interval, a skipped task event is
 * recorded, and data-table may not process samples as expected.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_process_samples(datatable_handle_t datatable_handle);

/**
 * @brief Deletes the data-table handle to frees up resources.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_delete(datatable_handle_t datatable_handle);

/**
 * @brief Converts a data-table to a `cJSON` object.
 * 
 * `cJSON` data-table object output example;
 * 
{
        "name": "1min_tbl",
        "process-interval":     "minute",
        "process-period":       1,
        "columns":      ["Record ID", "TS", "Pa_1-Min_Avg", "Ta_1-Min_Avg", "Ta_1-Min_Min", "Ta_1-Min_Max", "Hr_1-Min_Avg", "Td_1-Min_Avg", "Wd_1-Min_Avg", "Ws_1-Min_Avg"],
        "types":        ["id", "ts", "float", "float", "float", "float", "int16", "float", "float", "float"],
        "processes":    ["sample", "sample", "average", "average", "minimum", "maximum", "average", "average", "average", "average"],
        "rows": [
		[1, 61684380120, 1001.3500366210938, 22.350000381469727, 22.340000152587891, 22.360000610351562, 51, 20.350000381469727, 210, 1.4500000476837158], 
		[2, 61684380180, 1001.3533325195312, 22.35333251953125, 22.340000152587891, 22.360000610351562, 53, 20.353334426879883, 210, 1.4500000476837158], 
		[3, 61684380240, 1001.3583374023438, 22.358335494995117, 22.350000381469727, 22.3700008392334, 55, 20.358333587646484, 210, 1.4500000476837158], 
		[4, 61684380300, 1001.3417358398438, 22.341667175292969, 22.329999923706055, 22.350000381469727, 57, 20.341667175292969, 210, 1.4500000476837158]]
}
 *
 * @param[in] datatable_handle Data-table handle.
 * @param[out] datatable Data-table as a `cJSON` object.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_to_json(datatable_handle_t datatable_handle, cJSON **datatable);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __DATATABLE_H__
