#ifndef ROSPROCESSBUTTON_H
#define ROSPROCESSBUTTON_H
#include <QPushButton>
#include <QObject>


/** Button class for ButtonProcessAdministrators.
 * This class adds a context menu to a Button that is specific for ButtonProcessAdministrators.
 */
class RosProcessButton: public QPushButton
{
    Q_OBJECT
public:
    RosProcessButton(QWidget *parent);

protected:
    /**
     * Create a context menu.
     */
    void contextMenuEvent(QContextMenuEvent *event);

signals:
    /**
     * Kill the process.
     */
    void killPushed();

    /**
     * Start the process in a terminal.
     */
    void startInTerminalPushed();
};

#endif // ROSPROCESSBUTTON_H
