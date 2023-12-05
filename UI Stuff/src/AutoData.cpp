#include "../include/AutoData.h"

void AutoData::setTitle(char titleIN[MAX_TITLE_LENGTH]) {  // Set the auto title
  int i = 0;
  while(1) {
    title[i] = titleIN[i];
    i++;
    if(titleIN[i] == 0) {
      break;
    }
  }
  hasTitle = true;
}

void AutoData::setDescription(char descriptionIN[MAX_DESCRIPTION_LENGTH]) {  // Set the auto description
  int lineStartPos = 0;
  int lineEndPos = 0;
  bool tooLong = false;
  while (1) {  // Breaks at end of input
    while(1) {  // Breaks at end of line
      if(descriptionIN[lineEndPos] == '\n') {  // Used to check if the user put in their own new line calls
        break;
      }

      if(lineEndPos - lineStartPos > PER_LINE_MAX) {  // Checks to see if the current line would be too long
        tooLong = true;
      }

      if(tooLong && descriptionIN[lineEndPos] == ' ') {  // Roles back to find next space and makes that a new line-
        descriptionIN[lineEndPos] = '\n';
        break;
      }

      if(!tooLong) {
        lineEndPos++;
      } else if(tooLong) {
        lineEndPos--;
      }
    }

    for(int i = lineStartPos; i <= lineEndPos; i++) {  // Saves the next line to the description
      description[i] = descriptionIN[i];
    }
    lineStartPos = lineEndPos + 1;
    tooLong = false;

    if(descriptionIN[lineEndPos] == 0) {  // Checks to see if it is the end of the description
      break;
    }
  }
  hasDescription = true;
}

bool AutoData::printButtonData() {  // Prints all the data to the screen
  bool printedStuff = false;
  if(hasTitle) {
    Brain.Screen.setFont(mono60); //Set up parameters for title.
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print(title);  // Print the title
    printedStuff = true;
    if(hasDescription) {
      Brain.Screen.setFont(mono20);  //Set up description parameters.
      Brain.Screen.newLine(); 
      Brain.Screen.newLine();
      Brain.Screen.print(description);
    }
  }
  return(printedStuff);
}