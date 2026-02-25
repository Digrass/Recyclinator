#include <gui/wait_screen/WaitView.hpp>

WaitView::WaitView()
{

}

void WaitView::setupScreen()
{
    WaitViewBase::setupScreen();
}

void WaitView::tearDownScreen()
{
    WaitViewBase::tearDownScreen();
}

void WaitView::screenClicked()
{
    presenter->handleScreenClicked();
}
