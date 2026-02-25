#include <gui/down_screen/DownView.hpp>
#include <gui/down_screen/DownPresenter.hpp>
#include <gui/common/FrontendApplication.hpp>

DownPresenter::DownPresenter(DownView& v)
    : view(v)
{
}

void DownPresenter::activate()
{
    if (model)
     {
         model->bind(this);
     }
}

void DownPresenter::deactivate()
{
}

void DownPresenter::notifyActionSequenceComplete(OperationDirection actionDir)
{
    touchgfx_printf("UpPresenter: Action sequence for %d completed. Returning to MainScreen.\n", (int)actionDir);

    FrontendApplication* app = static_cast<FrontendApplication*>(Application::getInstance());
    if (app) {
        app->gotoMainScreenSlideTransitionNorth();
    }
}
