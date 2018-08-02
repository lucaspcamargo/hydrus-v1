#ifndef STATUSWIDGET_H
#define STATUSWIDGET_H

#include <QWidget>

#define BB_NO_TO_STRING
#include "droneblackboard.h"

namespace Ui {
class StatusWidget;
}

class StatusWidget : public QWidget
{
    Q_OBJECT
public:
    explicit StatusWidget(QWidget *parent = 0);
    ~StatusWidget();

signals:
    void rebootClicked();
    void haltClicked();
    void abortNavClicked();

public slots:
    void blackboardUpdate (Blackboard * bb );
    void connectionUpdate (bool connected);
    void addressChanged(QString addr);


private slots:
    void on_rebootBtn_clicked();

    void on_haltBtn_clicked();

    void on_abortNavBtn_clicked();

private:
    Ui::StatusWidget *ui;
    Blackboard m_previousBB;
    bool m_previousBBValid;
    bool m_didInitial;
};

#endif // STATUSWIDGET_H
