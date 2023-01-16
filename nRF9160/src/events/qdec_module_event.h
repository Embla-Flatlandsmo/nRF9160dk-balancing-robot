/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _QDEC_MODULE_EVENT_H_
#define _QDEC_MODULE_EVENT_H_

/**
 * @brief QDEC Event
 * @defgroup qdec_event QDEC Event
 * @{
 */

#include <app_event_manager.h>
#include <app_event_manager_profiler_tracer.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Enum containing types of qdec events emitted. */
    enum qdec_module_event_type
    {
        QDEC_EVT_DATA_READY,

        /** Events propagated when an error associated with the sensor device occurs. */
        QDEC_EVT_ERROR,
    };

    struct qdec_module_event
    {
        struct app_event_header header;
        /** Sensor module event type. */
        enum qdec_module_event_type type;
        int err;
        float travel;

    };

    APP_EVENT_TYPE_DECLARE(qdec_module_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _QDEC_MODULE_EVENT_H_ */
