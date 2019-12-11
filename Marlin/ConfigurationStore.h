#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"

void Config_ResetDefault();

#ifdef EEPROM_CHITCHAT
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {}
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings();
void Config_RetrieveSettings();
#ifdef POWER_LOSS_SAVE_TO_EEPROM
void EEPROM_Write_PLR(uint32_t lFPos=0, int iTPos=0, int iTPos1=0, int iT01=0, float fZPos=0.0, float fEPos=0.0);
void EEPROM_PRE_Write_PLR(uint32_t lFPos=0, int iBPos=0, int i_dual_x_carriage_mode=0, float f_duplicate_extruder_x_offset=0.0, float f_feedrate=0.0);
uint32_t EEPROM_Read_PLR_0();
String EEPROM_Read_PLR();
#endif //POWER_LOSS_SAVE_TO_EEPROM
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#endif//CONFIG_STORE_H
