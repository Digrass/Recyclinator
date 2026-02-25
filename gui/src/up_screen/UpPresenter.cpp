#include <gui/up_screen/UpView.hpp>
#include <gui/up_screen/UpPresenter.hpp>
#include <gui/common/FrontendApplication.hpp>

UpPresenter::UpPresenter(UpView& v)
    : view(v)
{
}

void UpPresenter::activate()
{
    if (model)
     {
         model->bind(this);
     }
}

void UpPresenter::deactivate()
{
}

void UpPresenter::notifyActionSequenceComplete(OperationDirection actionDir)
{
    touchgfx_printf("UpPresenter: Action sequence for %d completed. Returning to MainScreen.\n", (int)actionDir);

    FrontendApplication* app = static_cast<FrontendApplication*>(Application::getInstance());
    if (app) {
        app->gotoMainScreenSlideTransitionSouth();
    }
}
