#include <UI/UserInterface.h>

int main(int, char **) {
    VdbFields::UI::UserInterface::instance().testSkeletonize();
    VdbFields::UI::UserInterface::instance().display();
}