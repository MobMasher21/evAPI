#include "../include/AutoSelector.h"

AutoSelector::AutoSelector() {  // Constructor to set up page turning buttons
  pageBack = new Button(-1, &pageTurner);
  pageForward = new Button(1, &pageTurner);
  pageBack->setButtonPosition(165, 200);
  pageBack->setButtonSize(60, 30);
  pageForward->setButtonPosition(255, 200);
  pageForward->setButtonSize(60, 30);
  pageBack->setButtonIcon((bool*)arrows.previousPageArrow);
  pageForward->setButtonIcon((bool*)arrows.nextPageArrow);
}