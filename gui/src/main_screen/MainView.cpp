#include <gui/main_screen/MainView.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Unicode.hpp>

MainView::MainView()
{
}

void MainView::setupScreen()
{
	MainViewBase::setupScreen();
}

void MainView::tearDownScreen()
{
}

void MainView::upClicked()
{
    presenter->handleDirectionSelected(OperationDirection::UP);
}

void MainView::rightClicked()
{
    presenter->handleDirectionSelected(OperationDirection::RIGHT);
}

void MainView::downClicked()
{
    presenter->handleDirectionSelected(OperationDirection::DOWN);
}

void MainView::leftClicked()
{
    presenter->handleDirectionSelected(OperationDirection::LEFT);
}

void MainView::offClicked()
{
    presenter->handleOffScreen();
}

void MainView::updateCapacityDisplay(OperationDirection dir, float capacity)
{
	int progress_value = 0;
	if(capacity<99.5){
        progress_value = static_cast<int>(capacity + 0.5f);
	} else{
		progress_value = 100;
	}
    switch (dir)
    {
        case OperationDirection::UP:
            textProgress1.setValue(progress_value);
            break;
        case OperationDirection::RIGHT:
            textProgress2.setValue(progress_value);
            break;
        case OperationDirection::DOWN:
            textProgress3.setValue(progress_value);
            break;
        case OperationDirection::LEFT:
            textProgress4.setValue(progress_value);
            break;
        case OperationDirection::NONE:
        default:
            break;
    }
}
