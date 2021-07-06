#ifndef LANGUAGE_H
#define LANGUAGE_H

// NOTE: IF YOU CHANGE THIS FILE / MERGE THIS FILE WITH CHANGES
//
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRALCD" / "SDSUPPORT" #define IN "Configuration.h"
//   ==> ALSO TRY ALL AVAILABLE "LANGUAGE_CHOICE" OPTIONS

// Languages
// 1  English
// 2  Polish
// 3  French
// 4  German
// 5  Spanish
// 6  Russian
// 7  Italian
// 8  Portuguese
// 9  Finnish

#ifndef LANGUAGE_CHOICE
#define LANGUAGE_CHOICE 1 // Pick your language from the list above
#endif

#define MACHINE_NAME "TL 3D Printer"
#define FIRMWARE_URL "https://www.github.com/tenlog"

#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)

#if LANGUAGE_CHOICE == 1

// LCD Menu Messages
#define WELCOME_MSG MACHINE_NAME " Ready."
#define MSG_SD_INSERTED "Card inserted"
#define MSG_SD_REMOVED "Card removed"
#define MSG_MAIN "Main"
#define MSG_AUTOSTART "Autostart"
#define MSG_DISABLE_STEPPERS "Disable Steppers"
#define MSG_AUTO_HOME "Auto Home"
#define MSG_SET_ORIGIN "Set Origin"
#define MSG_PREHEAT_PLA "Preheat PLA"
#define MSG_PREHEAT_PLA_SETTINGS "Preheat PLA Conf"
#define MSG_PREHEAT_ABS "Preheat ABS"
#define MSG_PREHEAT_ABS_SETTINGS "Preheat ABS Conf"
#define MSG_COOLDOWN "Cooldown"
#define MSG_EXTRUDE "Extrude"
#define MSG_RETRACT "Retract"
#define MSG_MOVE_AXIS "Move Axis"
#define MSG_SPEED "Speed"
#define MSG_NOZZLE "Nozzle"
#define MSG_NOZZLE1 "Nozzle2"
#define MSG_NOZZLE2 "Nozzle3"
#define MSG_BED "Bed"
#define MSG_FAN_SPEED "Fan speed"
#define MSG_FLOW "Flow"
#define MSG_CONTROL "Control"
#define MSG_MIN " \002 Min"
#define MSG_MAX " \002 Max"
#define MSG_FACTOR " \002 Fact"
#define MSG_AUTOTEMP "Autotemp"
#define MSG_ON "On "
#define MSG_OFF "Off"
#define MSG_PID_P "PID-P"
#define MSG_PID_I "PID-I"
#define MSG_PID_D "PID-D"
#define MSG_PID_C "PID-C"
#define MSG_ACC "Accel"
#define MSG_VXY_JERK "Vxy-jerk"
#define MSG_VZ_JERK "Vz-jerk"
#define MSG_VE_JERK "Ve-jerk"
#define MSG_VMAX "Vmax "
#define MSG_X "x"
#define MSG_Y "y"
#define MSG_Z "z"
#define MSG_E "e"
#define MSG_VMIN "Vmin"
#define MSG_VTRAV_MIN "VTrav min"
#define MSG_AMAX "Amax "
#define MSG_A_RETRACT "A-retract"
#define MSG_XSTEPS "Xs./mm"
#define MSG_YSTEPS "Ys./mm"
#define MSG_ZSTEPS "Zs./mm"
#define MSG_ESTEPS "Es./mm"
#define MSG_RECTRACT "Rectract"
#define MSG_TEMPERATURE "Temperature"
#define MSG_MOTION "Motion"
#define MSG_CONTRAST "LCD contrast"
#define MSG_STORE_EPROM "Store memory"
#define MSG_LOAD_EPROM "Load memory"
#define MSG_RESTORE_FAILSAFE "Restore Failsafe"
#define MSG_REFRESH "Refresh"
#define MSG_WATCH "Info screen"
#define MSG_PREPARE "Prepare"
#define MSG_TUNE "Tune"
#define MSG_PAUSE_PRINT "Pause Print"
#define MSG_RESUME_PRINT "Resume Print"
#define MSG_STOP_PRINT_1 "Stop Print"
#define MSG_CARD_MENU "Print from SD"
#define MSG_NO_CARD "No Card"
#define MSG_DWELL "Sleep..."
#define MSG_USERWAIT "Wait for user..."
#define MSG_RESUMING "Resuming print"
#define MSG_NO_MOVE "No move."
#define MSG_KILLED "KILLED. "
#define MSG_STOPPED "STOPPED. "
#define MSG_CONTROL_RETRACT "Retract mm"
#define MSG_CONTROL_RETRACTF "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF "UnRet  F"
#define MSG_AUTORETRACT "AutoRetr."
#define MSG_FILAMENTCHANGE "Change filament"
#define MSG_INIT_SDCARD "Init. SD-Card"
#define MSG_CNG_SDCARD "Change SD-Card"

#define MSG_POWER_FAIL_RESUME "Power fail resume"
#define MSG_POWER_FAIL_RESUME_0 "Power fail resume..."
#define MSG_LOAD_FILAMENT "Load Filament"
#define MSG_UNLOAD_FILAMENT "Unload Filament"

// Serial Console Messages

#define MSG_Enqueing "enqueing \""
#define MSG_POWERUP "PowerUp"
#define MSG_EXTERNAL_RESET " External Reset"
#define MSG_BROWNOUT_RESET " Brown out Reset"
#define MSG_WATCHDOG_RESET " Watchdog Reset"
#define MSG_SOFTWARE_RESET " Software Reset"
#define MSG_MARLIN "Marlin "
#define MSG_AUTHOR " | Author: "
#define MSG_CONFIGURATION_VER " Last Updated: "
#define MSG_FREE_MEMORY " Free Memory: "
#define MSG_PLANNER_BUFFER_BYTES "  PlannerBufferBytes: "
#define MSG_OK "ok"
#define MSG_FILE_SAVED "Done saving file."
#define MSG_ERR_LINE_NO "Line Number is not Last Line Number+1, Last Line: "
#define MSG_ERR_CHECKSUM_MISMATCH "checksum mismatch, Last Line: "
#define MSG_ERR_NO_CHECKSUM "No Checksum with line number, Last Line: "
#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line: "
#define MSG_FILE_PRINTED "Done printing file"
#define MSG_BEGIN_FILE_LIST "Begin file list"
#define MSG_END_FILE_LIST "End file list"
#define MSG_M104_INVALID_EXTRUDER "M104 Invalid extruder "
#define MSG_M105_INVALID_EXTRUDER "M105 Invalid extruder "
#define MSG_M218_INVALID_EXTRUDER "M218 Invalid extruder "
#define MSG_ERR_NO_THERMISTORS "No thermistors - no temperature"
#define MSG_M109_INVALID_EXTRUDER "M109 Invalid extruder "
#define MSG_HEATING "Heating..."
#define MSG_HEATING_COMPLETE "Heating done."
#define MSG_BED_HEATING "Bed Heating."
#define MSG_BED_DONE "Bed done."
#define MSG_M115_REPORT "FIRMWARE_NAME:Marlin V1; Sprinter/grbl mashup for gen6 FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) "\n"
#define MSG_COUNT_X " Count X: "
#define MSG_ERR_KILLED "Printer halted. kill() called!"
#define MSG_ERR_STOPPED "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define MSG_RESEND "Resend: "
#define MSG_UNKNOWN_COMMAND "Unknown command: \""
#define MSG_ACTIVE_EXTRUDER "Active Extruder: "
#define MSG_INVALID_EXTRUDER "Invalid extruder"
#define MSG_X_MIN "x_min: "
#define MSG_X_MAX "x_max: "
#define MSG_Y_MIN "y_min: "
#define MSG_Y_MAX "y_max: "
#define MSG_Z_MIN "z_min: "
#define MSG_Z_MAX "z_max: "
#define MSG_M119_REPORT "Reporting endstop status"
#define MSG_ENDSTOP_HIT "TRIGGERED"
#define MSG_ENDSTOP_OPEN "open"
#define MSG_HOTEND_OFFSET "Hotend offsets:"

#define MSG_SD_CANT_OPEN_SUBDIR "Cannot open subdir"
#define MSG_SD_INIT_FAIL "SD init fail"
#define MSG_SD_VOL_INIT_FAIL "volume.init failed"
#define MSG_SD_OPENROOT_FAIL "openRoot failed"
#define MSG_SD_CARD_OK "SD card ok"
#define MSG_SD_WORKDIR_FAIL "workDir open failed"
#define MSG_SD_OPEN_FILE_FAIL "open failed, File: "
#define MSG_SD_FILE_OPENED "File opened: "
#define MSG_SD_SIZE " Size: "
#define MSG_SD_FILE_SELECTED "File selected"
#define MSG_SD_WRITE_TO_FILE "Writing to file: "
#define MSG_SD_PRINTING_BYTE "SD printing byte "
#define MSG_SD_NOT_PRINTING "Not SD printing"
#define MSG_SD_ERR_WRITE_TO_FILE "error writing to file"
#define MSG_SD_CANT_ENTER_SUBDIR "Cannot enter subdir: "

#define MSG_STEPPER_TOO_HIGH "Steprate too high: "
#define MSG_ENDSTOPS_HIT "endstops hit: "
#define MSG_ERR_COLD_EXTRUDE_STOP " cold extrusion prevented"
#define MSG_ERR_LONG_EXTRUDE_STOP " too long extrusion prevented"

#endif

#endif // ifndef LANGUAGE_H
