
/***************************************************************************
*   Copyright (C) 2020 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef UPLAY_H
#define UPLAY_H

/**
 * Class to start a thread that plays a 
 * sound track using default player (play).
 * The music runs until finished or when explicitly stopped.
 * \method start()   Start by calling 'start()'.
 * \method stopPlaying()  Stop process by calling 'stopPlaying()'.
 * */
class UPlay : public URun
{
public:
  /** destructor */
  ~UPlay()
  {
    stopPlaying();
  }
  /**
   * thread part, never call this directly, use start()
   * */
  virtual void run()
  { // soundfile is in 'fileToPlay'
    const int MSL = 300;
    char s[MSL];
    UTime t;
    t.now();
    // start the playing - with low priority (nice 14)
    int e = snprintf(s, MSL, "nice -n14 play -q -v%.1f %s", volume, fileToPlay);
    // -v0.1 gives low amplitude (10%)
    system(s);
//    printf("play finished\n");
    if (t.getTimePassed() < 1)
    {
      printf("# ---- file not found? (err=%d) '%s'\r\n", e, fileToPlay);
      printf("-- playing mp3 requires libsox-fmt-mp3 or libsox-fmt-all to be installed. \n");
    }
    runFinished();
  }
  /**
   * Test if we are playing anything */
  bool isPlaying()
  {
    int e = system("pgrep play");
    return e == 0;
  }
  /**
   * Kill the play process */
  void stopPlaying()
  { // kill the play process (if running)
    system("pkill play");
    // join the process (if running)
    stop();
  }
  /**
   * Name of file to play.
   * NB! the file must hard coded or in a percistent string.
   * */
  void setFile(const char * file)
  {
    fileToPlay = file;
  }
  /**
   * Set volume (0..100) */
  void setVolume(float level)
  {
    volume = level / 100.0;
  }
protected:
  // default music.mp3 is a symbolic link to some music
  const char * fileToPlay = "/home/local/Music/music.mp3";
  float volume = 0.1;
};

/**
 * Another class to also speak a text. */

class USay : public UPlay
{
public:
  /** destructor */
  ~USay()
  {
    stopSaying();
  }
  /**
   * Run the conversion from text to wav */
  void run()
  { // convert 
    int e = system("nice -n13 text2wave aa.txt -o aa.wav\n");
    if (e == 0)
    {
//       printf("USay:: all is fine\n");
      setFile("aa.wav");
      UPlay::run();
    }
    else
      printf("USay:: text2wave returned %d\n", e);
    saying = false;
  }
  /**
   * Test if we are playing anything */
  bool isSaying()
  {
    if (saying)
      return isPlaying();
    else
      return false;
  }
  /**
   * Kill the play process */
  void stopSaying()
  { // kill the play process (if running)
    stopPlaying();
  }
  /**
   * Say this sentence, i.e. convert to a wav file and play this file 
   * \param sentence is the text to say.
   * \param volume is an optional volume from 0% (silent) to 100%.
   * \return false if playing is busy, else true */
  bool say(const char * sentence, float volume = -1)
  {
    bool isOK = false;
    if (strlen(sentence) > 2 and not saying)
    {
      saying = true;
      FILE * a;
      if (volume >=0)
        setVolume(volume);
      a = fopen("aa.txt", "w");
      if (a != NULL)
      {
        fprintf(a, "%s\n", sentence);
        fclose(a);
        start();
        isOK = true;
      }
      else
      {
        printf("# USay::say: failed to save sentence to play\n");
        saying = false;
      }
    }
    else if (saying)
      printf("USay:: is busy (can not say '%s')\n", sentence);
    return isOK;
  }
  
  
protected:
  bool saying = false;
};

#endif
