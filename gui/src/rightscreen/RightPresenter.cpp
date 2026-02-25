#include <gui/right_screen/RightView.hpp>
#include <gui/right_screen/RightPresenter.hpp>
#include <gui/common/FrontendApplication.hpp>

RightPresenter::RightPresenter(RightView& v)
    : view(v)
{
}

void RightPresenter::activate()
{
    if (model)
     {
         model->bind(this);
     }
}

void RightPresenter::deactivate()
{
}

void RightPresenter::notifyActionSequenceComplete(OperationDirection actionDir)
{
    touchgfx_printf("UpPresenter: Action sequence for %d completed. Returning to MainScreen.\n", (int)actionDir);

    FrontendApplication* app = static_cast<FrontendApplication*>(Application::getInstance());
    if (app) {
        app->gotoMainScreenSlideTransitionWest();
    }
}
