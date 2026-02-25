#include <gui/main_screen/MainView.hpp>
#include <gui/main_screen/MainPresenter.hpp>
#include <gui/common/FrontendApplication.hpp> // 화면 전환을 위해 Application 클래스 접근

MainPresenter::MainPresenter(MainView& v)
    : view(v)
{
}

void MainPresenter::activate()
{
 if (model)
    {
        model->bind(this);
        view.updateCapacityDisplay(OperationDirection::UP, model->getCapacityForDirection(OperationDirection::UP));
        view.updateCapacityDisplay(OperationDirection::RIGHT, model->getCapacityForDirection(OperationDirection::RIGHT));
        view.updateCapacityDisplay(OperationDirection::DOWN, model->getCapacityForDirection(OperationDirection::DOWN));
        view.updateCapacityDisplay(OperationDirection::LEFT, model->getCapacityForDirection(OperationDirection::LEFT));
    }
}

void MainPresenter::deactivate()
{
}

void MainPresenter::notifyCapacityUpdated(OperationDirection dir, float newCapacity) {
    view.updateCapacityDisplay(dir, newCapacity);
}

void MainPresenter::handleDirectionSelected(OperationDirection dir)
{
    if (model)
    {
    	model->recordUserActivity();
        model->startServoSensorSequence(dir);
    }
}

void MainPresenter::handleOffScreen()
{
    if (model)
    {
    	model->recordUserActivity();
        model->forceTurnScreenOff();
    }
}

void MainPresenter::requestGoToWaitScreen()
{
    static_cast<FrontendApplication*>(Application::getInstance())->gotoWaitScreenNoTransition();
}
