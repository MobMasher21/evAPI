#ifndef AUTOSELECTOR_H_
#define AUTOSELECTOR_H_

#include "../../Common/include/evAPIBasicConfig.h"
#include "../../Common/include/evNamespace.h"
#include "Button.h"
#include "pageArrowsIcons.h"

#define MAX_BUTTON_COUNT 64

class AutoSelector {
  public:
    AutoSelector();  // Constructor to set up page turning buttons

  private:
    Button * buttonList[MAX_BUTTON_COUNT];  // List of all button objects in selector
    int buttonIDList[MAX_BUTTON_COUNT];  // List of all button IDs in selector
    int buttonCount;  // How many buttons there are
    int selectedButton;  // The button that is currently selected
    arrowIcons arrows;  // Structure with the arrow icons in it
    Button * pageBack;  // Button to move to next page
    Button * pageForward;  // Button to move to previous page
    int pageTurner;  // Used by page turn buttons to tell selector to go forward or back
    int selectedPage;  // The page that is being shown
    int highestUsedPage;  // The biggest page number that has buttons on it

    //===== Screen press stuff =====
    void pressed();  // Called when the screen is pressed
    bool prePressState;  // The previous state of the screen pressing, used for debouncing 

    //===== Thread items =====
    thread * selectorThread;  // Looping thread for the selector to run
    void threadFunction( void );  // Function that is run in the thread

};

#endif // AUTOSELECTOR_H_