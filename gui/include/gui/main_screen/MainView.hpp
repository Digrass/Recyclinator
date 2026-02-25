#ifndef MAIN_VIEW_HPP
#define MAIN_VIEW_HPP

#include <gui_generated/main_screen/MainViewBase.hpp>
#include <gui/main_screen/MainPresenter.hpp>
#include <gui/model/ModelCommon.hpp>

class MainView : public MainViewBase
{
public:
    MainView();
    virtual ~MainView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void upClicked() override; 
    virtual void rightClicked() override; 
    virtual void downClicked() override; 
    virtual void leftClicked() override; 
    virtual void offClicked() override;

    void updateCapacityDisplay(OperationDirection dir, float capacity);

protected:
};

#endif // MAIN_VIEW_HPP
