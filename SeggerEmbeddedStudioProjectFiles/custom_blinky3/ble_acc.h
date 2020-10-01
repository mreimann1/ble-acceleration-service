#ifndef BLE_ACC_H__
#define BLE_ACC_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/* UUIDs are used to identify characteristics and services*/
#define ACCELERATION_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
                                          0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define ACCELERATION_SERVICE_UUID               0x1400
#define ACCELERATION_VALUE_CHAR_UUID            0x1401
#define ACCELERATION_SIZE                 12 // 12 bytes will be used in acceleration readings

/**@brief   Macro for defining a ble_acc instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ACC_DEF(_name)                                                                          \
static ble_acc_t _name;                                                                             \

typedef enum
{
    ble_acc_EVT_DISCONNECTED,
    ble_acc_EVT_CONNECTED
} ble_acc_evt_type_t;

/**@brief Acceleration Service event. */
typedef struct
{
    ble_acc_evt_type_t evt_type;                                  /**< Type of event. */
} ble_acc_evt_t;

// Forward declaration of the ble_acc_t type.
typedef struct ble_acc_s ble_acc_t;

/**@brief Acceleration Service event handler type. */
typedef void (*ble_acc_evt_handler_t) (ble_acc_t * p_acc, ble_acc_evt_t * p_evt);

/**@brief Acceleration Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_acc_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Acceleration Service. */
    uint8_t                       initial_acceleration_value;           /**< Initial acceleration value @reduction_*/ 
    ble_srv_cccd_security_mode_t  acc_value_char_attr_md;     /**< Initial security level for Acceleration characteristics attribute */
} ble_acc_init_t;

/**@brief Acceleration Service structure. This contains various status information for the service. */
struct ble_acc_s
{
    ble_acc_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Acceleration Service. */
    uint16_t                      service_handle;                 /**< Handle of Acceleration Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      acc_value_handles;           /**< Handles related to the Acceleration Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

// Forward declaration of the ble_acc_t type.
typedef struct ble_acc_s ble_acc_t;

/**@brief Function for initializing the Acceleration Service.
 *
 * @param[out]  p_acc       Acceleration Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_acc_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_acc_init(ble_acc_t * p_acc, const ble_acc_init_t * p_acc_init);

/**@brief Function for updating the acceleration value.
 *
 * @details The application calls this function when the cutom value should be updated.
 *
 * @note 
 *       
 * @param[in]   p_acc          Acceleration Service structure.
 * @param[in]   acc_value      Pointer to the first byte of acceleration value buffer.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_acc_value_update(ble_acc_t * p_acc, uint8_t * acc_value);


#endif // BLE_ACC_H__