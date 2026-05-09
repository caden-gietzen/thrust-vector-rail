#include "py/runtime.h"
#include "encoder.h"

static mp_obj_t mp_encoder_set_drain_rate(mp_obj_t hz_obj) {
    encoder_set_drain_rate_c((uint32_t)mp_obj_get_int(hz_obj));
    return mp_const_none;
}

static mp_obj_t mp_encoder_get_drain_rate(void) {
    return mp_obj_new_int_from_uint(encoder_get_drain_rate_c());
}

static mp_obj_t mp_encoder_init_configured(size_t n_args, const mp_obj_t *args) {
    int pin_a = mp_obj_get_int(args[0]);
    int pin_b = mp_obj_get_int(args[1]);
    uint32_t max_step_rate = (uint32_t)mp_obj_get_int(args[2]);
    uint32_t drain_hz = (uint32_t)mp_obj_get_int(args[3]);

    bool ok = encoder_init_configured_c(pin_a, pin_b, max_step_rate, drain_hz);
    if (!ok) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid encoder init"));
    }
    return mp_const_none;
}

static mp_obj_t mp_encoder_init(mp_obj_t pin_a_obj, mp_obj_t pin_b_obj) {
    int pin_a = mp_obj_get_int(pin_a_obj);
    int pin_b = mp_obj_get_int(pin_b_obj);

    bool ok = encoder_init_c(pin_a, pin_b);
    if (!ok) {
        mp_raise_ValueError(MP_ERROR_TEXT("pin_b must equal pin_a + 1"));
    }
    return mp_const_none;
}

static mp_obj_t mp_encoder_get_count(void) {
    return mp_obj_new_int(encoder_get_count_c());
}

static mp_obj_t mp_encoder_zero(void) {
    encoder_zero_c();
    return mp_const_none;
}

static mp_obj_t mp_encoder_deinit(void) {
    encoder_deinit_c();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(mp_encoder_set_drain_rate_obj, mp_encoder_set_drain_rate);
MP_DEFINE_CONST_FUN_OBJ_0(mp_encoder_get_drain_rate_obj, mp_encoder_get_drain_rate);
MP_DEFINE_CONST_FUN_OBJ_2(mp_encoder_init_obj, mp_encoder_init);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_encoder_init_configured_obj, 4, 4, mp_encoder_init_configured);
MP_DEFINE_CONST_FUN_OBJ_0(mp_encoder_get_count_obj, mp_encoder_get_count);
MP_DEFINE_CONST_FUN_OBJ_0(mp_encoder_zero_obj, mp_encoder_zero);
MP_DEFINE_CONST_FUN_OBJ_0(mp_encoder_deinit_obj, mp_encoder_deinit);

static const mp_rom_map_elem_t encoder_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_encoder) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_encoder_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_count), MP_ROM_PTR(&mp_encoder_get_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_zero), MP_ROM_PTR(&mp_encoder_zero_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&mp_encoder_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drain_rate), MP_ROM_PTR(&mp_encoder_set_drain_rate_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drain_rate), MP_ROM_PTR(&mp_encoder_get_drain_rate_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_configured), MP_ROM_PTR(&mp_encoder_init_configured_obj) },
};

static MP_DEFINE_CONST_DICT(encoder_module_globals, encoder_module_globals_table);

const mp_obj_module_t encoder_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&encoder_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_encoder, encoder_user_cmodule);