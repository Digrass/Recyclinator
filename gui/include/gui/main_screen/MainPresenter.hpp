#ifndef MAIN_PRESENTER_HPP
#define MAIN_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <gui/model/Model.hpp>          // OperationDirection enum을 사용하기 위해 (또는 별도 헤더)
#include "gui/common/FrontendApplication.hpp"

using namespace touchgfx;

class MainView;

class MainPresenter : public Presenter, public ModelListener
{
public:
    MainPresenter(MainView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~MainPresenter() {};

    void handleDirectionSelected(OperationDirection dir);
    void handleOffScreen();
    virtual void notifyCapacityUpdated(OperationDirection dir, float newCapacity) override;
    virtual void requestGoToWaitScreen() override;

private:
    MainPresenter();

    MainView& view;
};

#endif // MAIN_PRESENTER_HPP
