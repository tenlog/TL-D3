#include "Marlin.h"
#include "cardreader.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "ConfigurationStore.h"

#ifdef SDSUPPORT

CardReader::CardReader()
{
    filesize = 0;
    sdpos = 0;
    sdprinting = 0;
    cardOK = false;
    saving = false;
    logging = false;
    autostart_atmillis = 0;
    workDirDepth = 0;
    memset(workDirParents, 0, sizeof(workDirParents));

    autostart_stilltocheck = true; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
    lastnr = 0;
    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER, HIGH);
#endif //SDPOWER

    autostart_atmillis = millis() + 5000;
}

char *createFilename(char *buffer, const dir_t &p) //buffer>12characters
{
    char *pos = buffer;
    for (uint8_t i = 0; i < 11; i++)
    {
        if (p.name[i] == ' ')
            continue;
        if (i == 8)
        {
            *pos++ = '.';
        }
        *pos++ = p.name[i];
    }
    *pos++ = 0;
    return buffer;
}

void CardReader::lsDive(const char *prepend, SdFile parent)
{
    dir_t p;
    uint8_t cnt = 0;

    while (parent.readDir(p, longFilename) > 0)
    {
        if (DIR_IS_SUBDIR(&p) && lsAction != LS_Count && lsAction != LS_GetFilename) // hence LS_SerialPrint
        {

            char path[13 * 2];
            char lfilename[13];
            createFilename(lfilename, p);

            path[0] = 0;
            if (strlen(prepend) == 0) //avoid leading / if already in prepend
            {
                strcat(path, "/");
            }
            strcat(path, prepend);
            strcat(path, lfilename);
            strcat(path, "/");

            //Serial.print(path);

            SdFile dir;
            if (!dir.open(parent, lfilename, O_READ))
            {
                if (lsAction == LS_SerialPrint)
                {
                    SERIAL_ECHO_START;
                    SERIAL_ECHOLNPGM(MSG_SD_CANT_OPEN_SUBDIR);
                    SERIAL_ECHOLN(lfilename);
                }
            }
            lsDive(path, dir);
            //close done automatically by destructor of SdFile
        }
        else
        {
            if (p.name[0] == DIR_NAME_FREE)
                break;
            if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.' || p.name[0] == '_')
                continue;
            if (longFilename[0] != '\0' &&
                (longFilename[0] == '.' || longFilename[0] == '_'))
                continue;
            if (p.name[0] == '.')
            {
                if (p.name[1] != '.')
                    continue;
            }

            if (!DIR_IS_FILE_OR_SUBDIR(&p))
                continue;
            filenameIsDir = DIR_IS_SUBDIR(&p);

            if (!filenameIsDir)
            {
                if (p.name[8] != 'G')
                    continue;
                if (p.name[9] == '~')
                    continue;
            }
            //if(cnt++!=nr) continue;
            createFilename(filename, p);
            if (lsAction == LS_SerialPrint)
            {
                SERIAL_PROTOCOL(prepend);
                SERIAL_PROTOCOLLN(filename);
            }
            else if (lsAction == LS_Count)
            {
                nrFiles++;
            }
            else if (lsAction == LS_GetFilename)
            {
                if (cnt == nrFiles)
                    return;
                cnt++;
                //SERIAL_PROTOCOL(prepend);
                //SERIAL_PROTOCOLLN(filename);
            }
        }
    }
}

void CardReader::ls()
{
    lsAction = LS_SerialPrint;
    if (lsAction == LS_Count)
        nrFiles = 0;

    root.rewind();
    lsDive("", root);
}

void CardReader::initsd()
{
    cardOK = false;
    if (root.isOpen())
        root.close();
#ifdef SDSLOW
    if (!card.init(SPI_HALF_SPEED, SDSS))
#else
    if (!card.init(SPI_FULL_SPEED, SDSS))
#endif
    {
        //if (!card.init(SPI_HALF_SPEED,SDSS))
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
        if(tl_TouchScreenType == 1)
            TLSTJC_printconstln(F("tStatus.txt=\"SD Card not detected\""));
        else if(tl_TouchScreenType == 0)
            DWN_Text(0x7100, 20, F("SD Card not detected"));
    }
    else if (!volume.init(&card))
    {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
    }
    else if (!root.openRoot(&volume))
    {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
    }
    else
    {
        cardOK = true;
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
        if(tl_TouchScreenType == 1)
            TLSTJC_printconstln(F("tStatus.txt=\"SD card OK\""));
        else if(tl_TouchScreenType = 0)
            DWN_Text(0x7100, 20, F("SD card OK"));
    }
    workDir = root;
    curDir = &root;
}

void CardReader::setroot()
{
    /*if(!workDir.openRoot(&volume))
    {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
    }*/
    workDir = root;

    curDir = &workDir;
}
void CardReader::release()
{
    sdprinting = 0;
    cardOK = false;
}

void CardReader::startFileprint()
{
    if (cardOK)
    {
        sdprinting = 1;
    }
}

void CardReader::pauseSDPrint()
{
    if (sdprinting == 1)
        sdprinting = 0;
}

void CardReader::openLogFile(char *name)
{
    logging = true;
    openFile(name, name, false); //By zyf
}

void CardReader::openFile(char *lngName, char *name, bool read, uint32_t startPos) //By zyf
{
    if (!cardOK)
        return;
    file.close();
    sdprinting = 0;

    SdFile myDir;
    curDir = &root;
    char *fname = name;

    char *dirname_start, *dirname_end;

    if (name[0] == '/')
    {
        dirname_start = strchr(name, '/') + 1;
        while (dirname_start > 0)
        {
            dirname_end = strchr(dirname_start, '/');
            //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
            //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
            if (dirname_end > 0 && dirname_end > dirname_start)
            {
                char subdirname[13];
                strncpy(subdirname, dirname_start, dirname_end - dirname_start);
                subdirname[dirname_end - dirname_start] = 0;
                SERIAL_ECHOLN(subdirname);
                if (!myDir.open(curDir, subdirname, O_READ))
                {
                    SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
                    SERIAL_PROTOCOL(subdirname);
                    SERIAL_PROTOCOLLNPGM(".");
                    sdprinting = 0;
                    return;
                }
                else
                {
                    //SERIAL_ECHOLN(subdirname);
                }

                curDir = &myDir;
                dirname_start = dirname_end + 1;
            }
            else // the reminder after all /fsa/fdsa/ is the filename
            {
                fname = dirname_start;
                //SERIAL_ECHOLN("remaider");
                //SERIAL_ECHOLN(fname);
                break;
            }
        }
    }
    else //relative path
    {
        curDir = &workDir;
        //SERIAL_PROTOCOL(workDir);
    }

    if (read)
    {
        if (file.open(curDir, fname, O_READ))
        {
            filesize = file.fileSize();
            SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
            SERIAL_PROTOCOL(fname);
            SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
            SERIAL_PROTOCOLLN(filesize);

//By Zyf
#ifdef POWER_LOSS_RECOVERY
            if (startPos > 0)
            {
                //SERIAL_PROTOCOLPGM("Print From ");
                //SERIAL_PROTOCOLLN(startPos);
                sdpos = startPos;
                setIndex(sdpos);
            }
            else
            {
                sdpos = 0;
            }
#else
            sdpos = 0;
#endif

            SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
            //lcd_setstatus(fname);

#ifdef POWER_LOSS_RECOVERY
            String strFName = fname;
            String strLFName = lngName;
            writeLastFileName(strLFName, strFName);
#if defined(POWER_LOSS_SAVE_TO_EEPROM)
            EEPROM_Write_PLR();
            EEPROM_PRE_Write_PLR();
#elif defined(POWER_LOSS_SAVE_TO_SDCARD)
            Write_PLR();
            PRE_Write_PLR();
#endif
#endif
        }
        else
        {
            SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
            SERIAL_PROTOCOL(fname);
            SERIAL_PROTOCOLLNPGM(".");
            sdprinting = 0;
        }
    }
    else
    { //write
        if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
        {
            SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
            SERIAL_PROTOCOL(fname);
            SERIAL_PROTOCOLLNPGM(".");
        }
        else
        {
            saving = true;
            SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
            SERIAL_PROTOCOLLN(name);
            //lcd_setstatus(fname);
        }
    }
}

void CardReader::removeFile(char *name)
{
    if (!cardOK)
        return;
    file.close();
    sdprinting = 0;

    SdFile myDir;
    curDir = &root;
    char *fname = name;

    char *dirname_start, *dirname_end;
    if (name[0] == '/')
    {
        dirname_start = strchr(name, '/') + 1;
        while (dirname_start > 0)
        {
            dirname_end = strchr(dirname_start, '/');
            //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
            //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
            if (dirname_end > 0 && dirname_end > dirname_start)
            {
                char subdirname[13];
                strncpy(subdirname, dirname_start, dirname_end - dirname_start);
                subdirname[dirname_end - dirname_start] = 0;
                SERIAL_ECHOLN(subdirname);
                if (!myDir.open(curDir, subdirname, O_READ))
                {
                    SERIAL_PROTOCOLPGM("open failed, File: ");
                    SERIAL_PROTOCOL(subdirname);
                    SERIAL_PROTOCOLLNPGM(".");
                    return;
                }
                else
                {
                    //SERIAL_ECHOLN("dive ok");
                }

                curDir = &myDir;
                dirname_start = dirname_end + 1;
            }
            else // the reminder after all /fsa/fdsa/ is the filename
            {
                fname = dirname_start;
                //SERIAL_ECHOLN("remaider");
                //SERIAL_ECHOLN(fname);
                break;
            }
        }
    }
    else //relative path
    {
        curDir = &workDir;
    }
    if (file.remove(curDir, fname))
    {
        SERIAL_PROTOCOLPGM("File deleted:");
        SERIAL_PROTOCOL(fname);
        sdpos = 0;
    }
    else
    {
        SERIAL_PROTOCOLPGM("Deletion failed, File: ");
        SERIAL_PROTOCOL(fname);
        SERIAL_PROTOCOLLNPGM(".");
    }
}

void CardReader::getStatus()
{
    if (cardOK)
    {
        SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
        SERIAL_PROTOCOL(sdpos);
        SERIAL_PROTOCOLPGM("/");
        SERIAL_PROTOCOLLN(filesize);
    }
    else
    {
        SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
    }
}
void CardReader::write_command(char *buf)
{
    char *begin = buf;
    char *npos = 0;
    char *end = buf + strlen(buf) - 1;

    file.writeError = false;
    if ((npos = strchr(buf, 'N')) != NULL)
    {
        begin = strchr(npos, ' ') + 1;
        end = strchr(npos, '*') - 1;
    }
    end[1] = '\r';
    end[2] = '\n';
    end[3] = '\0';
    file.write(begin);
    if (file.writeError)
    {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
    }
}

void CardReader::checkautostart(bool force)
{
    if (!force)
    {
        if (!autostart_stilltocheck)
            return;
        if (autostart_atmillis < millis())
            return;
    }
    autostart_stilltocheck = false;
    if (!cardOK)
    {
        initsd();
        if (!cardOK) //fail
            return;
    }

    char autoname[30];
    sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
    for (int8_t i = 0; i < (int8_t)strlen(autoname); i++)
        autoname[i] = tolower(autoname[i]);
    dir_t p;

    root.rewind();

    bool found = false;
    while (root.readDir(p, NULL) > 0)
    {
        for (int8_t i = 0; i < (int8_t)strlen((char *)p.name); i++)
            p.name[i] = tolower(p.name[i]);

        if (p.name[9] != '~') //skip safety copies
            if (strncmp((char *)p.name, autoname, 5) == 0)
            {
                char cmd[30];

                sprintf_P(cmd, PSTR("M23 %s"), autoname);
                enquecommand(cmd);
                enquecommand_P(PSTR("M24"));
                found = true;
            }
    }
    if (!found)
        lastnr = -1;
    else
        lastnr++;
}

void CardReader::closefile()
{
    file.sync();
    file.close();
    saving = false;
    logging = false;
}

void CardReader::getfilename(const uint8_t nr)
{
    curDir = &workDir;
    lsAction = LS_GetFilename;
    nrFiles = nr;
    curDir->rewind();
    lsDive("", *curDir);
}

uint16_t CardReader::getnrfilenames()
{
    curDir = &workDir;
    lsAction = LS_Count;
    nrFiles = 0;
    curDir->rewind();
    lsDive("", *curDir);
    //SERIAL_ECHOLN(nrFiles);
    return nrFiles;
}

void CardReader::chdir(const char *relpath)
{
    SdFile newfile;
    SdFile *parent = &root;

    if (workDir.isOpen())
        parent = &workDir;

    if (!newfile.open(*parent, relpath, O_READ))
    {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
        SERIAL_ECHOLN(relpath);
    }
    else
    {
        if (workDirDepth < MAX_DIR_DEPTH)
        {
            for (int d = ++workDirDepth; d--;)
                workDirParents[d + 1] = workDirParents[d];
            workDirParents[0] = *parent;
        }
        workDir = newfile;
    }
    //SERIAL_ECHOLN(relpath);
}

void CardReader::updir()
{
    if (workDirDepth > 0)
    {
        --workDirDepth;
        workDir = workDirParents[0];
        int d;
        for (int d = 0; d < workDirDepth; d++)
            workDirParents[d] = workDirParents[d + 1];
    }
}

void CardReader::printingHasFinished()
{
    st_synchronize();
    quickStop();
    file.close();
    sdprinting = 0;
    finishAndDisableSteppers(true); //By Zyf
    autotempShutdown();
}

#ifdef POWER_LOSS_RECOVERY

void CardReader::writeLastFileName(String LFName, String Value)
{
    if (!cardOK)
        return;

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLN.TXT";

    bool bFileExists = false;
    if (tf_file.open(*parent, tff, O_READ))
    {
        bFileExists = true;
        tf_file.close();
    }

    String sContent = "";
    char cAll[150];
    char cContent[50];

    sContent = LFName + "|";
    sContent += Value;
    sContent.toCharArray(cContent, 50);
    sprintf_P(cAll, PSTR("%s"), cContent);

    const char *arrFileContentNew = cAll;

    uint8_t O_TF = O_CREAT | O_EXCL | O_WRITE;
    if (bFileExists)
        O_TF = O_WRITE | O_TRUNC;

    if (tf_file.open(*parent, tff, O_TF))
    {
        tf_file.write(arrFileContentNew);
        tf_file.close();
    }
    else
    {
    }
}

///////////////////split
String CardReader::getSplitValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


String CardReader::isPowerLoss()
{
    if (!cardOK)
        return "";

    String sRet = "";

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLN.TXT";

    //Read File
    if (tf_file.open(*parent, tff, O_READ))
    {
        int16_t fS = tf_file.fileSize() + 1;
        char buf[255];
        char dim1[] = "\n";
        char *dim = dim1;
        int16_t n = tf_file.fgets(buf, fS, dim);
        String strFileContent = "";

        for (int i = 0; i < fS; i++)
        {
            if (buf[i] != '\0')
                strFileContent += buf[i];
        }

        if (strFileContent != "")
            sRet = strFileContent;
    }
    tf_file.close();

    if (sRet != "")
    {
        uint32_t lFPos = 0;
#if defined(POWER_LOSS_SAVE_TO_EEPROM)
        lFPos = EEPROM_Read_PLR_0();
#elif defined(POWER_LOSS_SAVE_TO_SDCARD)
        lFPos = Read_PLR_0();
#endif
        if (lFPos < 2048)
            sRet = "";
    }
    else
    {
        //TL_DEBUG_PRINT_LN(F("PLR File open fail."));
    }

    return sRet;
}

String CardReader::get_PLR()
{
    if (!cardOK)
        return "";

    String sRet = "";

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLN.TXT";

    //Read File
    if (tf_file.open(*parent, tff, O_READ))
    {
        int16_t fS = tf_file.fileSize() + 1;
        char buf[255];
        char dim1[] = "\n";
        char *dim = dim1;
        int16_t n = tf_file.fgets(buf, fS, dim);
        String strFileContent = "";

        for (int i = 0; i < fS; i++)
        {
            if (buf[i] != '\0')
                strFileContent += buf[i];
        }

        if (strFileContent != "")
        {
            sRet = strFileContent;
        }
    }
    tf_file.close();

    if (sRet != "")
    {
        String strRet = "";
#if defined(POWER_LOSS_SAVE_TO_EEPROM)
        strRet = EEPROM_Read_PLR();
#elif defined(POWER_LOSS_SAVE_TO_SDCARD)
        strRet = Read_PLR();
#endif
        sRet = sRet + "|" + strRet;
    }
    return sRet;
}

#ifdef POWER_LOSS_SAVE_TO_SDCARD
void CardReader::Write_PLR(uint32_t lFPos, int iTPos, int iTPos1, int iT01, float fZPos, float fEPos)
{
#ifdef POWER_LOSS_TRIGGER_BY_Z_LEVER
    if (lFPos == 0)
        fLastZ = 0.0;
#endif

#ifdef POWER_LOSS_TRIGGER_BY_E_COUNT
    if (lFPos == 0)
        lECount = POWER_LOSS_E_COUNT;
#endif

    if (!cardOK)
        return;

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLR.TXT";

    bool bFileExists = false;
    if (tf_file.open(*parent, tff, O_READ))
    {
        bFileExists = true;
        tf_file.close();
    }

    String sContent = "";
    char cAll[150];
    char cContent[15] = "";
    char cLine[15];
    const char *arrFileContentNew;

    uint32_t lFPos0 = sdpos;

    if (lFPos > 2048 && sdprinting == 1)
    {

        sContent.toCharArray(cContent, 12);
        float fValue = 0.0;

        ///////////// 0 = file Pos
        sContent = lFPos0;
        sContent.toCharArray(cContent, 12);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 1 = Temp0 Pos
        sContent = iTPos;
        sContent.toCharArray(cContent, 10);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 2 = Temp1 Pos
        sContent = iTPos1;
        sContent.toCharArray(cContent, 10);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 3 = T0T1
        sContent = iT01;
        sContent.toCharArray(cContent, 10);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 4 = Z Pos
        fValue = fZPos;
        dtostrf(fValue, 1, 2, cContent);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 5 = E Pos
        fValue = fEPos;
        dtostrf(fValue, 1, 2, cContent);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);
        arrFileContentNew = cAll;
    }
    else
    {
        arrFileContentNew = "0";
    }

    uint8_t O_TF = O_CREAT | O_EXCL | O_WRITE;
    if (bFileExists)
        O_TF = O_WRITE | O_TRUNC;

    if (tf_file.open(*parent, tff, O_TF))
    {
        tf_file.write(arrFileContentNew);
        tf_file.close();
    }
    else
    {
        //TL_DEBUG_PRINT_LN(F("Write Value Err "));
    }
}

bool b_PRE_Write_PLR_Done = false;
void CardReader::PRE_Write_PLR(uint32_t lFPos, int iBPos, int i_dual_x_carriage_mode, float f_duplicate_extruder_x_offset, float f_feedrate)
{
    if (!cardOK)
        return;

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PPLR.TXT";

    bool bFileExists = false;
    if (tf_file.open(*parent, tff, O_READ))
    {
        bFileExists = true;
        tf_file.close();
    }

    String sContent = "";
    char cAll[150];
    char cContent[15];
    char cLine[15];
    const char *arrFileContentNew;

    if (lFPos > 2048 && sdprinting == 1 && !b_PRE_Write_PLR_Done)
    {

        float fValue = 0.0;

        ///////////// 0 = Bed Temp
        sContent = iBPos;
        sContent.toCharArray(cContent, 10);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 1 = dual_x_carriage_mode
        sContent = dual_x_carriage_mode;
        sContent.toCharArray(cContent, 10);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////////  2 = duplicate_extruder_x_offset
        fValue = f_duplicate_extruder_x_offset;
        dtostrf(fValue, 1, 2, cContent);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        ///////////// 3 = feedrate
        fValue = f_feedrate;
        dtostrf(fValue, 1, 2, cContent);
        sprintf_P(cLine, PSTR("%s|"), cContent);
        strcat(cAll, cLine);

        arrFileContentNew = cAll;

        uint8_t O_TF = O_CREAT | O_EXCL | O_WRITE;
        if (bFileExists)
            O_TF = O_WRITE | O_TRUNC;

        if (tf_file.open(*parent, tff, O_TF))
        {
            tf_file.write(arrFileContentNew);
            tf_file.close();
        }
        else
        {
            //TL_DEBUG_PRINT_LN(F("New Value Err "));
        }
        b_PRE_Write_PLR_Done = true;
    }
}

uint32_t CardReader::Read_PLR_0()
{
    uint32_t lRet = 0;
    if (!cardOK)
        return 0;

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLR.TXT";

   //Read File
    if (tf_file.open(*parent, tff, O_READ))
    {
        int16_t fS = tf_file.fileSize() + 1;
        char buf[255];
        char dim1[] = "\n";
        char *dim = dim1;
        int16_t n = tf_file.fgets(buf, fS, dim);
        String strFileContent = "";

        for (int i = 0; i < fS; i++)
        {
            if (buf[i] != '\0')
                strFileContent += buf[i];
        }

        if (strFileContent != "")
        {
            lRet = atol(const_cast<char *>(getSplitValue(strFileContent, '|', 0).c_str()));
        }
    }
    
    tf_file.close();
    return lRet;
}

String CardReader::Read_PLR()
{
    String sRet = "";
    uint32_t lFP = 0;
    if (!cardOK)
        return "";

    SdFile tf_file;
    SdFile *parent = &root;
    const char *tff = "PLR.TXT";
String strFileContent = "";

    //Read File
    if (tf_file.open(*parent, tff, O_READ))
    {
        int16_t fS = tf_file.fileSize() + 1;
        char buf[255];
        char dim1[] = "\n";
        char *dim = dim1;
        int16_t n = tf_file.fgets(buf, fS, dim);

        for (int i = 0; i < fS; i++)
        {
            if (buf[i] != '\0')
                strFileContent += buf[i];
        }

        if (strFileContent != "")
        {
            lFP = atol(const_cast<char *>(getSplitValue(strFileContent, '|', 0).c_str()));
        }
    }
    
    tf_file.close();
    if (lFP > 2048)
    {
        const char *tff = "PPLR.TXT";
        String strFileContent1 = "";

        //Read File
        if (tf_file.open(*parent, tff, O_READ))
        {
            int16_t fS = tf_file.fileSize() + 1;
            char buf[255];
            char dim1[] = "\n";
            char *dim = dim1;
            int16_t n = tf_file.fgets(buf, fS, dim);

            for (int i = 0; i < fS; i++)
            {
                if (buf[i] != '\0')
                    strFileContent1 += buf[i];
            }

            if (strFileContent1 != "")
            {
                String sFC = strFileContent;
                sRet = sFC + "255|0|0|" + strFileContent1;
            }
        }
        tf_file.close();
    }
    return sRet;
}

#endif //#ifdef POWER_LOSS_SAVE_TO_SDCARD
#endif //POWER_LOSS_RECOVERY

#endif //SDSUPPORT
