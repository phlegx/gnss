
#include "gnss.h"

#define UBX_FRAME_SIZE 8
#ifdef __cplusplus
extern "C" {
#endif

/** Enums
*/
enum Command {
    POWER_ON,
    POWER_OFF,
    MON_VER,
    ENABLE_UBX,
    RESTART, // mbed conflict with RESET
    CUSTOMER,
    AVAILABLE_CONFIG
};
/** The reset modes
*/
enum Start {
    HOT,
    COLD,
    WARM,
    MAX_MODE
};

/** The operation modes
*/
enum Powermodes {

    CONSERVATIVE_CONTINOUS,
    AGGRESSIVE_CONTINUOS,
    SEMI_CONTINOUS,
    FULL_POWER,
    FULL_POWER_BLOCK_LEVEL,
    FULL_POWER_BUILDING_LEVEL,
    AVAILABLE_OPERATION
};


class GnssOperations : public GnssSerial {

    //GnssSerial constructor can be called here to configure different baud rate
    //Constructor not required at the moment
    //GnssOperations();

public:

    /** Enable GNSS receiver UBX-NAV-PVT messages
     *  Navigation Position Velocity Time Solution
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_nav_pvt();

    /** Enable GNSS receiver UBX-STATUS messages
     *  Receiver Navigation Status
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_nav_status();

    /** Enable GNSS receiver UBX-NAV-SAT messages
     *  Satellite Information
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_nav_sat();

    /** Enable GNSS receiver UBX-NAV-SOL messages
     * Navigation Solution Information
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_nav_sol();

    /** Disable GNSS receiver UBX-NAV-PVT messages
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int disable_ubx_nav_pvt();

    /** Enable GNSS receiver UBX-NAV5 messages
     *  Navigation Engine Settings
     *  @param  uint 	acc 	Defines positioning accuracy
     *  @return int             1: Successful
     *                          0: Failure
     */
    int enable_ubx_nav5(unsigned int acc);

    /** Enable GNSS receiver UBX-NAVX5 messages
     *  Navigation Engine Settings
     *  @return int             1: Successful
     *                          0: Failure
     */
    int enable_ubx_navx5();

    /** Enable GNSS receiver UBX-CFG-ODO messages
     *  Odometer, Low-speed COG Engine Settings
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_odo();

    /** Disable GNSS receiver UBX-CFG-ODO messages
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int disable_ubx_odo();

    /** Enable GNSS receiver UBX-NAV-ODO messages
     *  Odometer, Low-speed COG Engine Settings
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_nav_odo();

    /** Disable GNSS receiver UBX-NAV-ODO messages
     *  Odometer, Low-speed COG Engine Settings
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int disable_ubx_nav_odo();

    /** Enable GNSS receiver UBX-LOG-BATCH messages
     *  Batched data
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int enable_ubx_batch_feature();

    /** Disable GNSS receiver UBX-LOG-BATCH messages
     *  Batched data
     *  @param void
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int disable_ubx_batch_feature();

    /** Configure GNSS receiver batching feature
     *  Get/Set data batching configuration
     *  @param 	tUBX_CFG_BATCH
     *  @return int 	1: Successful
     * 	                0: Failure
     */
    int cfg_batch_feature(tUBX_CFG_BATCH *obj);

    /** Configure GNSS receiver power mode
     *  Power mode setup
     *  @param 	Powermodes     SEMI_CONTINOUS
     *                          AGGRESSIVE_CONTINUOS
     *                          CONSERVATIVE_CONTINOUS
     *                          FULL_POWER
     *                          FULL_POWER_BLOCK_LEVEL
     *          minimumAcqTime  boolean
     *
     *  @return int             1: Successful
     * 	                       0: Failure
     */
    int cfg_power_mode(Powermodes power_mode, bool minimumAcqTime);

    /** Method to poll the GNSS configuration
     *  @param 	void
     *  @return bool    true: 	Successful
     * 	                false:	Failure
     */
    bool verify_gnss_mode();

    /** Configure GNSS startup mode
     *  Power mode setup
     *  @param 	start_mode      0: Hot Start
     *                          1: Warm Start
     *                          2: Cold Start
     *
     *  @return int             1: Successful
     *                          0: Failure
     */
    int start_mode(int start_mode);

    /** Send char to GNSS receiver
     *  @param 	char
     *  @return void
     */
    void send_to_gnss(char);

    /** Power On GNSS receiver
     *
     *  @return void
     */
    void power_on_gnss();

};
#ifdef __cplusplus
}
#endif
