#include "gnss_operations.h"

#ifndef UBLOX_WEARABLE_FRAMEWORK
#define SEND_LOGGING_MESSAGE printf
#else
#include "MessageView.h"
#endif

#define FIRST_BYTE 0x000000FF
#define SECOND_BYTE 0x0000FF00
#define THIRD_BYTE 0x00FF0000
#define FOURTH_BYTE 0xFF000000
#define RETRY 5

#define EXTRACT_BYTE(INDEX, BYTE, VALUE) ((VALUE & BYTE) >> (INDEX*8))

/**
 *
 * Enable UBX-NAV-PVT using UBX-CFG-MSG
 * @param return 	SUCCESS: 1
 * 					FAILURE: 0
 */
int GnssOperations::enable_ubx_nav_pvt()
{
    int conf = RETRY;
    unsigned char enable_ubx_nav_pvt[]= {0x01, 0x07, 0x01};
    conf = RETRY;
    int length =0;

    while(conf)
    {

        length = GnssSerial::sendUbx(0x06, 0x01, enable_ubx_nav_pvt, sizeof(enable_ubx_nav_pvt));
        if(length >= (int)(sizeof(enable_ubx_nav_pvt) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-PVT was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-NAV-PVT...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

int GnssOperations::enable_ubx_nav_status() {
    int conf = RETRY;
    unsigned char enable_ubx_nav_status[]= {0x01, 0x03, 0x01};
    conf = RETRY;
    int length =0;

    while(conf)
    {

        length = GnssSerial::sendUbx(0x06, 0x01, enable_ubx_nav_status, sizeof(enable_ubx_nav_status));
        if(length >= (int)(sizeof(enable_ubx_nav_status) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-STATUS was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-NAV-STATUS...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;

}

int GnssOperations::enable_ubx_nav_sat() {
    int conf = RETRY;
    unsigned char enable_ubx_nav_sat[]= {0x01, 0x35, 0x01};
    conf = RETRY;
    int length =0;

    while(conf)
    {

        length = GnssSerial::sendUbx(0x06, 0x01, enable_ubx_nav_sat, sizeof(enable_ubx_nav_sat));
        if(length >= (int)(sizeof(enable_ubx_nav_sat) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-STATUS was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-NAV-STATUS...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;

}

int GnssOperations::enable_ubx_nav_sol() {
    int conf = RETRY;
    unsigned char enable_ubx_nav_status[]= {0x01, 0x06, 0x0A};
    conf = RETRY;
    int length =0;

    while(conf)
    {

        length = GnssSerial::sendUbx(0x06, 0x01, enable_ubx_nav_status, sizeof(enable_ubx_nav_status));
        if(length >= (int)(sizeof(enable_ubx_nav_status) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-STATUS was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-NAV-STATUS...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;

}


/**
 *
 * Disable UBX-NAV-PVT
 * @param return 	SUCCESS: 1
 * 					FAILURE: 0
 */
int GnssOperations::disable_ubx_nav_pvt()
{
    int conf = RETRY;
    unsigned char enable_ubx_nav_pvt[]= {0x01, 0x07, 0x00};
    conf = RETRY;
    int length =0;

    while(conf)
    {

        length = GnssSerial::sendUbx(0x06, 0x01, enable_ubx_nav_pvt, sizeof(enable_ubx_nav_pvt));
        if(length >= (int)(sizeof(enable_ubx_nav_pvt) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-PVT was disabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("disabling UBX-NAV-PVT...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

int GnssOperations::enable_ubx_nav5(unsigned int acc)
{
    int conf = RETRY;
    conf = RETRY;
    int length =0;
    //convert unsigned int acc to hex
    //ask if positioning mask or time accuracy mask
    unsigned char 	ubx_cfg_nav5[]= {0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                                     0x0A, 0x00, 0xFA, 0x00,0xFA, 0x00, (unsigned char)EXTRACT_BYTE(0, FIRST_BYTE, acc), (unsigned char)EXTRACT_BYTE(1, SECOND_BYTE, acc),
                                     0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00
                                   };

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x24, ubx_cfg_nav5, sizeof(ubx_cfg_nav5));
        if(length >= (int)(sizeof(ubx_cfg_nav5) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("ubx_cfg_nav5 was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling ubx_cfg_nav5...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

int GnssOperations::enable_ubx_navx5()
{
    int conf = RETRY;
    conf = RETRY;
    int length =0;
    //convert unsigned int acc to hex
    //ask if positioning mask or time accuracy mask
    unsigned char   ubx_cfg_navx5[]= {0x28, 0x00, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0x02, 0x00, 0x00, 0x03, 0x02, 0x03, 0x20, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0x90,
                                      0x07, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x64, 0x64, 0x00, 0x00, 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0xF7
                                     };

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x23, ubx_cfg_navx5, sizeof(ubx_cfg_navx5));
        if(length >= (int)(sizeof(ubx_cfg_navx5) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("ubx_cfg_navx5 was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling ubx_cfg_navx5...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

/**
 * Enabling UBX-ODOMETER using UBX-CFG-ODO
 * @param return 	SUCCESS: 1
 * 					FAILURE: 0
 *
 */
int GnssOperations::enable_ubx_odo()
{
    int conf = RETRY;
    unsigned char ubx_cfg_odo[]= {0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x19, 0x46, 0x19, 0x66, 0x0A, 0x32, 0x00,
                                  0x00, 0x99, 0x4C, 0x00, 0x00
                                 };
    conf = RETRY;
    int length =0;

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x1E, ubx_cfg_odo, sizeof(ubx_cfg_odo));
        if(length >= (int)(sizeof(ubx_cfg_odo) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-ODO was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-ODO...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

int GnssOperations::disable_ubx_odo()
{
    int conf = RETRY;
    unsigned char ubx_cfg_odo[]= {0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x19, 0x46, 0x19, 0x66, 0x0A, 0x32, 0x00,
                                  0x00, 0x99, 0x4C, 0x00, 0x00
                                 };
    conf = RETRY;
    int length =0;

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x1E, ubx_cfg_odo, sizeof(ubx_cfg_odo));
        if(length >= (int)(sizeof(ubx_cfg_odo) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-ODO was disabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("disabling UBX-ODO...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}
/**
 * Enabling UBX-NAV-ODO messages using UBX-CFG-MSG
 * @param return 	SUCCESS: 1
 * 					FAILURE: 0
 *
 */
int GnssOperations::enable_ubx_nav_odo()
{
    int conf = RETRY;
    unsigned char ubx_nav_odo[]= {0x01, 0x09, 0x01};
    conf = RETRY;
    int length =0;

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x01, ubx_nav_odo, sizeof(ubx_nav_odo));
        if(length >= (int)(sizeof(ubx_nav_odo) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-ODO was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enabling UBX-NAV-ODO...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

/**
 * Disabling UBX-NAV-ODO messages using UBX-CFG-MSG
 * @param return 	SUCCESS: 1
 * 					FAILURE: 0
 *
 */
int GnssOperations::disable_ubx_nav_odo()
{
    int conf = RETRY;
    unsigned char ubx_nav_odo[]= {0x01, 0x09, 0x00};
    conf = RETRY;
    int length =0;

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x01, ubx_nav_odo, sizeof(ubx_nav_odo));
        if(length >= (int)(sizeof(ubx_nav_odo) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX-NAV-ODO was disabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("disabling UBX-NAV-ODO...\r\n");
            conf = conf - 1;
        }
    }

    return (conf == 0) ? 0 : 1;
}

int GnssOperations::enable_ubx_batch_feature()
{
    int conf = RETRY;
    unsigned char enable_ubx_log_batch[]= {0x00, 0x0D, 0x0A, 0x00, 0x07, 0x00, 0x00, 0x01};
    conf = RETRY;
    int length =0;

    //Disable NAV-ODO and NAV-PVT
    disable_ubx_nav_odo();
    disable_ubx_nav_pvt();

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x93, enable_ubx_log_batch, sizeof(enable_ubx_log_batch));
        if(length >= (int)(sizeof(enable_ubx_log_batch) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX_LOG_BATCH was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enable ubx_batch_log...\r\n");
            conf = conf - 1;
        }
    }
    return (conf == 0) ? 0 : 1;
}

int GnssOperations::disable_ubx_batch_feature()
{
    int conf = RETRY;
    unsigned char enable_ubx_log_batch[]= {0x00, 0x0C, 0x0A, 0x00, 0x07, 0x00, 0x00, 0x01};
    conf = RETRY;
    int length =0;

    //Enable NAV-ODO and NAV-PVT
    enable_ubx_nav_odo();
    enable_ubx_nav_pvt();

    while(conf)
    {
        length = GnssSerial::sendUbx(0x06, 0x93, enable_ubx_log_batch, sizeof(enable_ubx_log_batch));
        if(length >= (int)(sizeof(enable_ubx_log_batch) + UBX_FRAME_SIZE))
        {
            SEND_LOGGING_MESSAGE("UBX_LOG_BATCH was enabled\r\n");
            thread_sleep_for(500);
            break;
        }
        else
        {
            SEND_LOGGING_MESSAGE("enable ubx_batch_log...\r\n");
            conf = conf - 1;
        }
    }
    return (conf == 0) ? 0 : 1;
}

/**
 *
 * Configuring UBX-LOG-BATCH with UBX-CFG-BATCH
 *
 * @param obj struct containing the data to be send in payload
 * @param return 	SUCCESS: 1
 * 					FAIL:    0
 *
 */
int GnssOperations::cfg_batch_feature(tUBX_CFG_BATCH *obj)
{
    int length =0;
    const unsigned char cfg_batch_feature[] = {0x00, 0x01, (unsigned char)EXTRACT_BYTE(0, FIRST_BYTE, obj->bufSize),
                                               (unsigned char) EXTRACT_BYTE(1, SECOND_BYTE, obj->bufSize), (unsigned char) EXTRACT_BYTE(0, FIRST_BYTE, obj->notifThrs),
                                               (unsigned char) EXTRACT_BYTE(1, SECOND_BYTE, obj->notifThrs), obj->pioId, 0x00
                                              };

    length = GnssSerial::sendUbx(0x06, 0x93, cfg_batch_feature, sizeof(cfg_batch_feature));

    return (length >= (int)(sizeof(cfg_batch_feature) + UBX_FRAME_SIZE)) ? 1 : 0;
}

/*
 *  Power mode configuration for GNSS receiver
 *
 *	Pending: Need to send extended power management configuration messages (UBX-CFG-PM2)
 *
 *
 */
int GnssOperations::cfg_power_mode(Powermodes power_mode, bool minimumAcqTimeZero)
{
    int length = 0;
    const int minimumAcqTime_index = 22;
    unsigned char semi_continuous_pms[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char semi_continuous_pm2[] = {0x02, 0x06, 0x00, 0x00, 0x02, 0x00, 0x43, 0x01, 0x10, 0x27, 0x00, 0x00, 0x10,
                                           0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x2C, 0x01, 0x00, 0x00, 0xCF, 0x40, 0x00,
                                           0x00, 0x87, 0x5A, 0xA4, 0x46, 0xFE, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                          };
    unsigned char semi_continuous_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    unsigned char aggresive_continuous_pms[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char aggresive_continuous_pm2[] = {0x02, 0x06, 0x00, 0x00, 0x02, 0x00, 0x43, 0x01, 0xE8, 0x03, 0x00, 0x00,
                                                0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x2C, 0x01, 0x00, 0x00, 0xCF, 0x40,
                                                0x00, 0x00, 0x87, 0x5A, 0xA4, 0x46, 0xFE, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                               };
    unsigned char aggressive_continuous_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    unsigned char conservative_continuous_pms[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char conservative_continuous_pm2[] = {0x02, 0x06, 0x00, 0x00, 0x00, 0x00, 0x43, 0x01, 0xE8, 0x03, 0x00, 0x00,
                                                   0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x2C, 0x01, 0x00, 0x00, 0xCF, 0x41,
                                                   0x00, 0x00, 0x88, 0x6A, 0xA4, 0x46, 0xFE, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                                  };
    unsigned char conservative_continuous_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    unsigned char full_power_pms[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char full_power_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    unsigned char full_power_block_level_pms[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char full_power_block_level_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    unsigned char full_power_building_level_pms[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char full_power_building_level_rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

    switch (power_mode)
    {
    case SEMI_CONTINOUS:
        SEND_LOGGING_MESSAGE("Configuring SEMI_CONTINOUS");
        length = GnssSerial::sendUbx(0x06, 0x86, semi_continuous_pms, sizeof(semi_continuous_pms));
        thread_sleep_for(500);

        if(minimumAcqTimeZero) {
            semi_continuous_pm2[minimumAcqTime_index] = 0x00;
            semi_continuous_pm2[minimumAcqTime_index + 1] = 0x00;
        }

        length = GnssSerial::sendUbx(0x06, 0x3B, semi_continuous_pm2, sizeof(semi_continuous_pm2));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, semi_continuous_rate, sizeof(semi_continuous_rate));
        thread_sleep_for(500);
        break;

    case AGGRESSIVE_CONTINUOS:
        SEND_LOGGING_MESSAGE("Configuring AGGRESSIVE_CONTINUOS");
        length = GnssSerial::sendUbx(0x06, 0x86, aggresive_continuous_pms, sizeof(aggresive_continuous_pms));
        thread_sleep_for(500);

        if(minimumAcqTimeZero) {
            semi_continuous_pm2[minimumAcqTime_index] = 0x00;
            semi_continuous_pm2[minimumAcqTime_index + 1] = 0x00;
        }

        length = GnssSerial::sendUbx(0x06, 0x3B, aggresive_continuous_pm2, sizeof(aggresive_continuous_pm2));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, aggressive_continuous_rate, sizeof(aggressive_continuous_rate));
        thread_sleep_for(500);
        break;

    case CONSERVATIVE_CONTINOUS:
        SEND_LOGGING_MESSAGE("Configuring CONSERVATIVE_CONTINOUS");
        length = GnssSerial::sendUbx(0x06, 0x86, conservative_continuous_pms, sizeof(conservative_continuous_pms));
        thread_sleep_for(500);

        if(minimumAcqTimeZero) {
            semi_continuous_pm2[minimumAcqTime_index] = 0x00;
            semi_continuous_pm2[minimumAcqTime_index + 1] = 0x00;
        }

        length = GnssSerial::sendUbx(0x06, 0x3B, conservative_continuous_pm2, sizeof(conservative_continuous_pm2));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, conservative_continuous_rate, sizeof(conservative_continuous_rate));
        thread_sleep_for(500);
        break;

    case FULL_POWER:
        SEND_LOGGING_MESSAGE("Configuring FULL_POWER");
        length = GnssSerial::sendUbx(0x06, 0x86, full_power_pms, sizeof(full_power_pms));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, full_power_rate, sizeof(full_power_rate));
        thread_sleep_for(500);
        break;
    case FULL_POWER_BLOCK_LEVEL:
        SEND_LOGGING_MESSAGE("Configuring FULL_POWER_BLOCK_LEVEL");
        length = GnssSerial::sendUbx(0x06, 0x86, full_power_block_level_pms, sizeof(full_power_block_level_pms));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, full_power_block_level_rate, sizeof(full_power_block_level_rate));
        thread_sleep_for(500);
        break;
    case FULL_POWER_BUILDING_LEVEL:
        SEND_LOGGING_MESSAGE("Configuring FULL_POWER_BUILDING_LEVEL");
        length = GnssSerial::sendUbx(0x06, 0x86, full_power_building_level_pms, sizeof(full_power_building_level_pms));
        thread_sleep_for(500);
        length = GnssSerial::sendUbx(0x06, 0x08, full_power_building_level_rate, sizeof(full_power_building_level_rate));
        thread_sleep_for(500);
        break;
    case AVAILABLE_OPERATION:
    default : {
        SEND_LOGGING_MESSAGE("Invalid power mode");
    }
    break;
    }

    return (length >= (int)(sizeof(semi_continuous_pms) + UBX_FRAME_SIZE)) ? 1 : 0;
}

bool GnssOperations::verify_gnss_mode() {

    unsigned char CFG_PMS[] = {0xB5, 0x62, 0x06, 0x86, 0x00, 0x00, 0x8c, 0xAA};
    unsigned char CFG_PM2[] = {0xB5, 0x62, 0x06, 0x3B, 0x00, 0x00, 0x41, 0xC9};
    unsigned char CFG_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    unsigned char CFG_NAV5[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
    unsigned char CFG_NAVX5[] = {0xB5, 0x62, 0x06, 0x23, 0x00, 0x00, 0x29, 0x81};

    this->_send(CFG_PMS, sizeof(CFG_PMS));
    thread_sleep_for(500);

    this->_send(CFG_PM2, sizeof(CFG_PM2));
    thread_sleep_for(500);

    this->_send(CFG_RATE, sizeof(CFG_RATE));
    thread_sleep_for(500);

    this->_send(CFG_NAV5, sizeof(CFG_NAV5));
    thread_sleep_for(500);

    this->_send(CFG_NAVX5, sizeof(CFG_NAVX5));
    thread_sleep_for(500);

    return true;
}

/**
 *  GNSS start modes (Hot/Warm/Cold start)
 *
 *	@param return 	SUCCESS: 1
 * 					FAILURE:    0
 *
 */
int GnssOperations::start_mode(int start_mode)
{
    int length = 0;
    unsigned char hot_start[] = {0x00, 0x00, 0x02, 0x00};
    unsigned char warm_start[] = {0x01, 0x00, 0x02, 0x00};
    unsigned char cold_start[] = {0xFF, 0xFF, 0x02, 0x00};

    switch (start_mode)
    {
    case HOT:
        length = GnssSerial::sendUbx(0x06, 0x04, hot_start, sizeof(hot_start));
        break;

    case WARM:
        length = GnssSerial::sendUbx(0x06, 0x04, warm_start, sizeof(warm_start));
        break;

    case COLD:
        length = GnssSerial::sendUbx(0x06, 0x04, cold_start, sizeof(cold_start));
        break;
    }

    return (length >= (int)(sizeof(hot_start) + UBX_FRAME_SIZE)) ? 1 : 0;
}

void GnssOperations::send_to_gnss(char rChar)
{
    GnssSerial::putc(rChar);
}

void GnssOperations::power_on_gnss()
{
    GnssSerial::_powerOn();
}
