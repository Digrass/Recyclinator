#ifndef WAITPRESENTER_HPP
#define WAITPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class WaitView;

class WaitPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    WaitPresenter(WaitView& v);

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

    virtual ~WaitPresenter() {}

    void handleScreenClicked();
    virtual void requestGoToMainScreen() override;

private:
    WaitPresenter();

    WaitView& view;
};

#endif // WAITPRESENTER_HPP
