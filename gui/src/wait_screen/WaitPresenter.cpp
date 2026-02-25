#include <gui/wait_screen/WaitView.hpp>
#include <gui/wait_screen/WaitPresenter.hpp>

WaitPresenter::WaitPresenter(WaitView& v)
    : view(v)
{

}

void WaitPresenter::activate()
{

}

void WaitPresenter::deactivate()
{

}

void WaitPresenter::handleScreenClicked()
{
    if (model)
    {
    	model->recordUserActivity();
        model->turnScreenOnInternal();
    }
}

void WaitPresenter::requestGoToMainScreen()
{
    static_cast<FrontendApplication*>(Application::getInstance())->gotoMainScreenNoTransition();
}

