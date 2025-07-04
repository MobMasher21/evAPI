#include "evAPI/autoSelector/AutoData.h"

namespace evAPI {
void AutoData::setTitle(char titleIN[MAX_TITLE_LENGTH]) {  // Set the auto title
  int i = 0;
  while (1) {
    title[i] = titleIN[i];
    i++;
    if (titleIN[i] == '\0') {
      break;
    }
  }
  hasTitle = true;
}

void AutoData::setDescription(char descriptionIN[MAX_DESCRIPTION_LENGTH]) {  // Set the auto description
  int nullTermPos = 0;
  int lineStartPos = 0;
  int lineEndPos = 0;
  int lineCharacterCount;
  bool tooLong = false;
  bool break1 = true;
  bool break2 = true;

  // finds where the NULL terminator at the end of the string is
  while (descriptionIN[nullTermPos] != '\0') {
    nullTermPos++;
  }
  nullTermPos++;

  while (break1) {  // Breaks at end of input
    break2 = true;  // Reset the inner loop break var
    tooLong = false;
    lineCharacterCount = 1;
    while (break2) {                            // Breaks at end of line
      if (descriptionIN[lineEndPos] == '\n') {  // Used to check if the user put in their own new line calls
        break2 = false;
        break;
      }

      if (!tooLong) {
        lineEndPos++;
        lineCharacterCount++;
      } else if (tooLong) {
        lineEndPos--;
        lineCharacterCount--;
      }

      if (lineCharacterCount > PER_LINE_MAX) {  // Checks to see if the current line would be too long
        tooLong = true;
      }

      if (tooLong && descriptionIN[lineEndPos] == ' ') {  // Roles back to find next space and makes that a new line-
        descriptionIN[lineEndPos] = '\n';
        break2 = false;
      }

      // checks to see if they reached the end
      if (lineEndPos == nullTermPos) {
        break1 = false;
        break2 = false;
        break;
      }
    }

    for (int i = lineStartPos; i <= lineEndPos; i++) {  // Saves the next line to the description
      description[i] = descriptionIN[i];
    }
    lineStartPos = lineEndPos + 1;
    lineEndPos++;
  }
  hasDescription = true;
}

bool AutoData::printButtonData() {  // Prints all the data to the screen
  bool didPrintedStuff = false;
  if (hasTitle) {
    Brain.Screen.setFont(vex::fontType::mono60);  // Set up parameters for title.
    Brain.Screen.setFillColor(vex::color::transparent);
    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(title);  // Print the title
    Brain.Screen.newLine();
    didPrintedStuff = true;
    if (hasDescription) {
      Brain.Screen.setFont(vex::fontType::mono20);  // Set up description parameters.
      Brain.Screen.setCursor(4, 1);
      //while (1) {
      for(int i = 0; description[i] != '\0'; i++) {
        if (description[i] == '\n') {
          Brain.Screen.newLine();
        } else {
          Brain.Screen.print("%c", description[i]);
        }
      }
    }
  }
  return (didPrintedStuff);
}

std::string AutoData::getTitle() {
  return std::string(title);
}

std::string AutoData::getDescription() {
  return std::string(description);
}
}  // namespace evAPI