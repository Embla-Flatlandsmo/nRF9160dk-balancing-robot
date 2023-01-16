/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>

#include "qdec_module_event.h"

static char *get_evt_type_str(enum qdec_module_event_type type)
{
    switch (type)
    {
    case QDEC_EVT_DATA_READY:
        return "QDEC_EVT_MOVEMENT_DATA_READY";
    case QDEC_EVT_ERROR:
        return "QDEC_EVT_ERROR";
    default:
        return "Unknown event";
    }
}

static void profile_qdec_module_event(struct log_event_buf *buf,
                                      const struct app_event_header *aeh)
{
}

APP_EVENT_INFO_DEFINE(qdec_module_event,
                      ENCODE(),
                      ENCODE(),
                      profile_qdec_module_event);

static void log_qdec_event(const struct app_event_header *eh)
{
    const struct qdec_module_event *event = cast_qdec_module_event(eh);

    if (event->type == QDEC_EVT_ERROR)
    {
        APP_EVENT_MANAGER_LOG(eh, "%s - Error code %d",
            get_evt_type_str(event->type), event->err);
    }
    else if (event->type == QDEC_EVT_DATA_READY)
    {
        APP_EVENT_MANAGER_LOG(eh, "%s - Travel: %.2f",
                          get_evt_type_str(event->type),
                          event->travel
                          );
    }
    else
    {
        APP_EVENT_MANAGER_LOG(eh, "%s", get_evt_type_str(event->type));
    }
}

APP_EVENT_TYPE_DEFINE(qdec_module_event,
                    log_qdec_event,
                    &qdec_module_event_info,
                    APP_EVENT_FLAGS_CREATE(
                        IF_ENABLED(CONFIG_QDEC_EVENTS_LOG,
                        (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));