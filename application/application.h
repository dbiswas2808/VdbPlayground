#include <UI/user_interface.h>

class Application {
  Application() {}

  void run() { UI::UserInterface::instance()->show(); }
}