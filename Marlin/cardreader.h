#ifndef CARDREADER_H
#define CARDREADER_H

#ifdef SDSUPPORT

#define MAX_DIR_DEPTH 10

#include "SdFile.h"
enum LsAction
{
	LS_SerialPrint,
	LS_Count,
	LS_GetFilename
};
class CardReader
{
public:
	CardReader();

	void initsd();
	void write_command(char *buf);
	//files auto[0-9].g on the sd card are performed in a row
	//this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

	void checkautostart(bool x);
	void openFile(char *lngName, char *name, bool read, uint32_t startPos = 0); //By zyf
	void openLogFile(char *name);
	void removeFile(char *name);
	void closefile();
	void release();
	void startFileprint();
	void pauseSDPrint();
	void getStatus();
	void printingHasFinished();

	void getfilename(const uint8_t nr);
	uint16_t getnrfilenames();

	void ls();
	void chdir(const char *relpath);
	void updir();
	void setroot();

	FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
	FORCE_INLINE bool eof() { return sdpos >= filesize; };
	FORCE_INLINE int16_t get()
	{
		sdpos = file.curPosition();
		return (int16_t)file.read();
	};
	FORCE_INLINE void setIndex(long index)
	{
		sdpos = index;
		file.seekSet(index);
	};
	FORCE_INLINE uint8_t percentDone()
	{
		if (!isFileOpen())
			return 0;
		if (filesize)
			return sdpos / ((filesize + 99) / 100);
		else
			return 0;
	};
	FORCE_INLINE char *getWorkDirName()
	{
		workDir.getFilename(filename);
		return filename;
	};

#ifdef POWER_LOSS_RECOVERY
	void writeLastFileName(String LFName, String Value);

#ifdef POWER_LOSS_SAVE_TO_SDCARD
	void Write_PLR(uint32_t lFPos = 0, int iTPos = 0, int iTPos1 = 0, int iT01 = 0, float fZPos = 0.0, float fEPos = 0.0);
	void PRE_Write_PLR(uint32_t lFPos = 0, int iBPos = 0, int i_dual_x_carriage_mode = 0, float f_duplicate_extruder_x_offset = 0.0, float f_feedrate = 0.0);
	uint32_t Read_PLR_0();
	String Read_PLR();
#endif

	String getSplitValue(String data, char separator, int index);

	String isPowerLoss();
	String get_PLR();
#endif

public:
	bool heating;
	bool saving;
	bool logging;
	int sdprinting;
	bool cardOK;
	char filename[13];
	char longFilename[LONG_FILENAME_LENGTH];
	bool filenameIsDir;
	int lastnr; //last number of the autostart;
	uint32_t sdpos;
	uint32_t filesize;

private:
	SdFile root, *curDir, workDir, workDirParents[MAX_DIR_DEPTH];
	uint16_t workDirDepth;
	Sd2Card card;
	SdVolume volume;
	SdFile file;
	unsigned long autostart_atmillis;

	bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

	LsAction lsAction; //stored for recursion.
	int16_t nrFiles;   //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
	char *diveDirName;
	void lsDive(const char *prepend, SdFile parent);
};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting == 1)

#if (SDCARDDETECT > -1)
#ifdef SDCARDDETECTINVERTED
#define IS_SD_INSERTED (READ(SDCARDDETECT) != 0)
#else
#define IS_SD_INSERTED (READ(SDCARDDETECT) == 0)
#endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
#define IS_SD_INSERTED true
#endif

#else
#define IS_SD_PRINTING (false)
#endif //SDSUPPORT
#endif //CARDREADER_H
