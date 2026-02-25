#include <gui/left_screen/LeftView.hpp>
#include <gui/left_screen/LeftPresenter.hpp>
#include <gui/common/FrontendApplication.hpp>

LeftPresenter::LeftPresenter(LeftView& v)
    : view(v)
{
}

void LeftPresenter::activate()
{
    if (model)
     {
         model->bind(this);
     }
}

void LeftPresenter::deactivate()
{
}

void LeftPresenter::notifyActionSequenceComplete(OperationDirection actionDir)
{
    touchgfx_printf("UpPresenter: Action sequence for %d completed. Returning to MainScreen.\n", (int)actionDir);

    FrontendApplication* app = static_cast<FrontendApplication*>(Application::getInstance());
    if (app) {
        app->gotoMainScreenSlideTransitionEast();
    }
}
