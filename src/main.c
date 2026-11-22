#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>

/* Идентификаторы эндпоинтов */
#define EP_FULL_OPEN    1
#define EP_WICKET       2
#define EP_REED_SWITCH  3

/* Параметры устройства для Zigbee2MQTT */
#define INIT_BASIC_APP_VERSION       01
#define INIT_BASIC_STACK_VERSION     10
#define INIT_BASIC_HW_VERSION        01
#define INIT_BASIC_MANUF_NAME        "DIY"
#define INIT_BASIC_MODEL_ID          "SuperMini_Gate_v1"
#define INIT_BASIC_DATE_CODE         "20261122"
#define INIT_BASIC_POWER_SOURCE      ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Оборудование из Devicetree */
static const struct gpio_dt_spec led_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec btn_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec relay1_spec = GPIO_DT_SPEC_GET(DT_ALIAS(relay_full), gpios);
static const struct gpio_dt_spec relay2_spec = GPIO_DT_SPEC_GET(DT_ALIAS(relay_wicket), gpios);
static const struct gpio_dt_spec reed_spec = GPIO_DT_SPEC_GET(DT_ALIAS(reed_switch), gpios);

static struct gpio_callback btn_cb_data;
static struct gpio_callback reed_cb_data;

/* Контексты атрибутов ZCL */
typedef struct {
    zb_uint8_t zcl_version;
    zb_uint8_t app_version;
    zb_uint8_t stack_version;
    zb_uint8_t hw_version;
    zb_char_t mf_name[32];
    zb_char_t model_id[32];
    zb_char_t date_code[16];
    zb_uint8_t power_source;
    zb_uint8_t location_id[17];
    zb_uint8_t ph_env;
} basic_attr_t;

typedef struct {
    zb_uint16_t identify_time;
} identify_attr_t;

typedef struct {
    zb_bool_t on_off;
} on_off_attr_t;

typedef struct {
    zb_bool_t out_of_service;
    zb_bool_t present_value;
    zb_uint8_t status_flags;
} binary_input_attr_t;

static basic_attr_t dev_basic_attr;
static identify_attr_t dev_identify_attr;
static on_off_attr_t relay1_on_off_attr;
static on_off_attr_t relay2_on_off_attr;
static binary_input_attr_t reed_bi_attr;

/* Декларация кластеров ZCL */
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
    &dev_basic_attr.zcl_version,
    &dev_basic_attr.app_version,
    &dev_basic_attr.stack_version,
    &dev_basic_attr.hw_version,
    dev_basic_attr.mf_name,
    dev_basic_attr.model_id,
    dev_basic_attr.date_code,
    &dev_basic_attr.power_source,
    dev_basic_attr.location_id,
    &dev_basic_attr.ph_env);

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &dev_identify_attr.identify_time);
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(on_off_attr_list_1, &relay1_on_off_attr.on_off);
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(on_off_attr_list_2, &relay2_on_off_attr.on_off);

ZB_ZCL_DECLARE_BINARY_INPUT_ATTRIB_LIST(bi_attr_list,
    &reed_bi_attr.out_of_service,
    &reed_bi_attr.present_value,
    &reed_bi_attr.status_flags);

/* Списки кластеров по эндпоинтам */
zb_zcl_cluster_desc_t ep1_clusters[] = {
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t), basic_attr_list, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), identify_attr_list, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_ARRAY_SIZE(on_off_attr_list_1, zb_zcl_attr_t), on_off_attr_list_1, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
};

zb_zcl_cluster_desc_t ep2_clusters[] = {
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), identify_attr_list, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_ARRAY_SIZE(on_off_attr_list_2, zb_zcl_attr_t), on_off_attr_list_2, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
};

zb_zcl_cluster_desc_t ep3_clusters[] = {
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), identify_attr_list, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
    ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_BINARY_INPUT_BASIC, ZB_ZCL_ARRAY_SIZE(bi_attr_list, zb_zcl_attr_t), bi_attr_list, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID),
};

/* Дескрипторы эндпоинтов */
ZB_AF_SIMPLE_DESC_TYPE(3, 0) simple_desc_ep1 = {
    EP_FULL_OPEN, ZB_AF_HA_PROFILE_ID, ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 1, 0, 3, 0,
    { ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_CLUSTER_ID_ON_OFF }
};

ZB_AF_SIMPLE_DESC_TYPE(2, 0) simple_desc_ep2 = {
    EP_WICKET, ZB_AF_HA_PROFILE_ID, ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 1, 0, 2, 0,
    { ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_CLUSTER_ID_ON_OFF }
};

ZB_AF_SIMPLE_DESC_TYPE(2, 0) simple_desc_ep3 = {
    EP_REED_SWITCH, ZB_AF_HA_PROFILE_ID, ZB_HA_SIMPLE_SENSOR_DEVICE_ID, 1, 0, 2, 0,
    { ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_CLUSTER_ID_BINARY_INPUT_BASIC }
};

ZB_AF_DECLARE_ENDPOINT_DESC(ep1_desc, EP_FULL_OPEN, ZB_AF_HA_PROFILE_ID, 0, NULL,
    ZB_ZCL_ARRAY_SIZE(ep1_clusters, zb_zcl_cluster_desc_t), ep1_clusters,
    (zb_af_simple_desc_1_1_t *)&simple_desc_ep1, 0, NULL, 0, NULL);

ZB_AF_DECLARE_ENDPOINT_DESC(ep2_desc, EP_WICKET, ZB_AF_HA_PROFILE_ID, 0, NULL,
    ZB_ZCL_ARRAY_SIZE(ep2_clusters, zb_zcl_cluster_desc_t), ep2_clusters,
    (zb_af_simple_desc_1_1_t *)&simple_desc_ep2, 0, NULL, 0, NULL);

ZB_AF_DECLARE_ENDPOINT_DESC(ep3_desc, EP_REED_SWITCH, ZB_AF_HA_PROFILE_ID, 0, NULL,
    ZB_ZCL_ARRAY_SIZE(ep3_clusters, zb_zcl_cluster_desc_t), ep3_clusters,
    (zb_af_simple_desc_1_1_t *)&simple_desc_ep3, 0, NULL, 0, NULL);

zb_af_endpoint_desc_t *gate_ep_list[] = { &ep1_desc, &ep2_desc, &ep3_desc };
zb_af_device_ctx_t gate_ctx = {
    3,
    gate_ep_list
};



/* --- Отложенные задачи (Workqueues) --- */

/* 1. Отключение Реле 1 через 500 мс */
static void relay1_off_handler(struct k_work *work)
{
    gpio_pin_set_dt(&relay1_spec, 0);
    zb_uint8_t val = ZB_FALSE;
    zb_zcl_set_attr_val(EP_FULL_OPEN, ZB_ZCL_CLUSTER_ID_ON_OFF,
                        ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                        &val, ZB_FALSE);
}
K_WORK_DELAYABLE_DEFINE(relay1_off_work, relay1_off_handler);

/* 2. Отключение Реле 2 через 500 мс */
static void relay2_off_handler(struct k_work *work)
{
    gpio_pin_set_dt(&relay2_spec, 0);
    zb_uint8_t val = ZB_FALSE;
    zb_zcl_set_attr_val(EP_WICKET, ZB_ZCL_CLUSTER_ID_ON_OFF,
                        ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                        &val, ZB_FALSE);
}
K_WORK_DELAYABLE_DEFINE(relay2_off_work, relay2_off_handler);

/* 3. Антидребезг и отправка статуса геркона */
static void reed_work_handler(struct k_work *work)
{
    /* gpio_pin_get_dt возвращает 1, если геркон замкнут на GND (ворота ЗАКРЫТЫ).
       Для Z2M contact sensor: false (0) = closed, true (1) = open */
    int closed = gpio_pin_get_dt(&reed_spec);
    zb_bool_t is_open = (closed == 1) ? ZB_FALSE : ZB_TRUE;

    zb_zcl_set_attr_val(EP_REED_SWITCH, ZB_ZCL_CLUSTER_ID_BINARY_INPUT_BASIC,
                        ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                        (zb_uint8_t *)&is_open, ZB_FALSE);
}
K_WORK_DELAYABLE_DEFINE(reed_work, reed_work_handler);

/* 4. Сброс привязки Zigbee (Factory Reset) при удержании кнопки 5 сек */
static void reset_work_handler(struct k_work *work)
{
    if (gpio_pin_get_dt(&btn_spec) == 1) {
        zb_bdb_reset_via_local_action(0);
    }
}
K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);

/* Прерывания GPIO */
static void reed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_reschedule(&reed_work, K_MSEC(200));
}

static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get_dt(&btn_spec) == 1) {
        k_work_reschedule(&reset_work, K_SECONDS(5));
    } else {
        k_work_cancel_delayable(&reset_work);
    }
}

/* Обработчик входящих ZCL-команд */
static void zcl_device_cb(zb_bufid_t bufid)
{
    zb_zcl_device_callback_param_t *p_device_cb_param = ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

    if (p_device_cb_param->device_cb_id == ZB_ZCL_SET_ATTR_VALUE_CB_ID) {
        if (p_device_cb_param->cb_param.set_attr_value_param.cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF) {
            zb_uint8_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data8;
            zb_uint8_t ep = p_device_cb_param->endpoint;

            if (value == ZB_TRUE) {
                if (ep == EP_FULL_OPEN) {
                    gpio_pin_set_dt(&relay1_spec, 1);
                    k_work_reschedule(&relay1_off_work, K_MSEC(500));
                } else if (ep == EP_WICKET) {
                    gpio_pin_set_dt(&relay2_spec, 1);
                    k_work_reschedule(&relay2_off_work, K_MSEC(500));
                }
            }
        }
    }
    p_device_cb_param->status = RET_OK;
}

/* Обработчик событий стека Zigbee */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t *p_sg_p = NULL;
    zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

    switch (sig) {
    case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == RET_OK) {
            gpio_pin_set_dt(&led_spec, 1); /* Горит — в сети */
            k_work_reschedule(&reed_work, K_MSEC(500)); /* Отправляем текущий статус ворот */
        } else {
            gpio_pin_set_dt(&led_spec, 0);
            bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
        }
        break;
    default:
        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
        break;
    }

    if (bufid) {
        zb_buf_free(bufid);
    }
}

static void init_clusters_attr(void)
{
    dev_basic_attr.zcl_version = ZB_ZCL_VERSION;
    dev_basic_attr.app_version = INIT_BASIC_APP_VERSION;
    dev_basic_attr.stack_version = INIT_BASIC_STACK_VERSION;
    dev_basic_attr.hw_version = INIT_BASIC_HW_VERSION;
    ZB_ZCL_SET_STRING_VAL(dev_basic_attr.mf_name, INIT_BASIC_MANUF_NAME, ZB_ZCL_STRING_CONST_SIZE(INIT_BASIC_MANUF_NAME));
    ZB_ZCL_SET_STRING_VAL(dev_basic_attr.model_id, INIT_BASIC_MODEL_ID, ZB_ZCL_STRING_CONST_SIZE(INIT_BASIC_MODEL_ID));
    ZB_ZCL_SET_STRING_VAL(dev_basic_attr.date_code, INIT_BASIC_DATE_CODE, ZB_ZCL_STRING_CONST_SIZE(INIT_BASIC_DATE_CODE));
    dev_basic_attr.power_source = INIT_BASIC_POWER_SOURCE;

    relay1_on_off_attr.on_off = ZB_FALSE;
    relay2_on_off_attr.on_off = ZB_FALSE;

    reed_bi_attr.out_of_service = ZB_FALSE;
    reed_bi_attr.present_value = ZB_FALSE;
    reed_bi_attr.status_flags = 0;
}

int main(void)
{
    /* Инициализация GPIO */
    gpio_pin_configure_dt(&led_spec, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&relay1_spec, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&relay2_spec, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&btn_spec, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn_spec, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb_data, btn_isr, BIT(btn_spec.pin));
    gpio_add_callback(btn_spec.port, &btn_cb_data);

    gpio_pin_configure_dt(&reed_spec, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&reed_spec, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&reed_cb_data, reed_isr, BIT(reed_spec.pin));
    gpio_add_callback(reed_spec.port, &reed_cb_data);

    /* Инициализация Zigbee */
    init_clusters_attr();
    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);
    ZB_AF_REGISTER_DEVICE_CTX(&gate_ctx);

    zigbee_erase_persistent_storage(ZB_FALSE);
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    zigbee_enable();

    return 0;
}
