#include "time_impl.h"

/*
 compileTime returns the compile date and time as a time_t value.
 This code is taken directly from JChristensen timezone clock example
 found here: https://github.com/JChristensen/Timezone/blob/master/examples/Clock/Clock.ino
*/
time_t compileTime()
{
    const uint8_t COMPILE_TIME_DELAY = 8;
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char chMon[4], *m;
    tmElements_t tm;

    strncpy(chMon, compDate, 3);
    chMon[3] = '\0';
    m = strstr(months, chMon);
    tm.Month = ((m - months) / 3 + 1);

    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);
    time_t t = makeTime(tm);
    return t + COMPILE_TIME_DELAY;
}

/*
 toUTC converts local time to its UTC
 equivalent based off the UTC_OFFSET value
 defined in time_impl.h
*/
time_t toUTC(time_t local)
{
    return local - UTC_OFFSET * 3600L;
}