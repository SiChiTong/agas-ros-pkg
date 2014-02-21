#ifndef BUTTONPROCESSADMINISTRATOR_H
#define BUTTONPROCESSADMINISTRATOR_H

#include <QPushButton>
#include <QProcess>
#include <QMetaType>
#include "rosprocessbutton.h"

class ButtonProcessAdministrator : public QObject
{
    Q_OBJECT
public:

    ButtonProcessAdministrator(QObject *parent = 0);

    /**
      Init the ButtonProcessAdministrator with a name.
      @param QString name The name of the
      */
    ButtonProcessAdministrator(QString getName, QString command_, QStringList arguments_, QObject *parent = 0);

    /**
      A copyconstructor.
      */
    ButtonProcessAdministrator(const ButtonProcessAdministrator &copy);
    ~ButtonProcessAdministrator();

    QString getName() const;
    QString getCommand()const;
    QStringList getArguments()const;

    /**
      Connects a Button with the Admin.
      */
    void addButton(QAbstractButton* button);

    /**
      Connects a QPushButton with the Admin.
      @param
      */
    void addButton(QPushButton* button);

    /**
      Connects a RosProcessButton with the Admin.
      @param
      */
    void addButton(RosProcessButton* button);

    /**
     Sets the command and its arguments.
     */
    void setCommand(QString, QStringList);

    /**
      Returns the current state of the internal process.
      */
    QProcess::ProcessState getState()const;

   /**
     Getter for combinedString.
     @return combinedString
     */
    QString getCombinedString()const;

    /**
      Getter for stderrString.
      @return stderrString
      */
    QString getStderrString()const;

    /**
      Getter for stdcoutString.
      @return stdcoutString
      */
    QString getStdcoutString()const;

    /**
    Starts the process.
    */
    void stopProcess();

    /**
    Stops the process.
    */
    void startProcess();

    /**
     * Extended enum for the process state. With shutting down state.
     */
    enum ProcessState{
        NotRunning,
        Starting,
        Running,
        ShuttingDown
    };


private:
    void setName(QString name_);
    void setButtonText(QString);
    void connectProcess();
    ProcessState current_process_state_;


public slots:
    void handleClick();
    void handleProcessError(QProcess::ProcessError error);
    void startInExternalTerminal();

private slots:
    /**
      Is called when new data from stdout is available.
      */
    void readStdoutProcess();

    /**
      Is called when new data from stderr is available.
      */
    void readStderrProcess();

    /**
      Sets the buttontext to start.
      */
    void setButtonTextStop();

    /**
      Set the buttontext to stop.
      */
    void setButtonTextStart();

    /**
     * Sets the buttontext to starting.
     */
    void setButtonTextStarting();

    /**
     * Sets the buttontext to starting.
     */
    void setButtonTextShuttingDown();

    /**
      Handles the state change of a process.
      */
    void handleProcessStateChange(QProcess::ProcessState state);

signals:
    /**
      This signal is emitted if the combinedString has changed.
      */
    void combinedStringChanged(QString text);
    void combinedStringChanged();
    /**
      This signal is emitted if the stderrString has changed.
      */
    void stderrStringChanged(QString text);
    void stderrStringChanged();

    /**
      This signal is emitted if the stdcoutString has changed.
      */
    void stdcoutStringChanged(QString text);

    /**
      This signal is emitted if the stdcoutString has changed.
      */
    void stdcoutStringChanged();

private:

     /** \brief Contains the buttons which are administrated. */
    QVector<QAbstractButton*> buttons_;

     /** \brief The name of the process. */
    QString name_;

     /** \brief The QProcess object for starting a process. */
    QProcess* process_;

     /** \brief The command which is executed. */
    QString command_;


     /** \brief The arguments which are passed to the command. */
    QStringList arguments_;

    /** \brief Contains the cout output from the process. */
    QString stdcout_string_;
    /** \brief Contains the cerr output from the process. */
    QString stderr_string_;
    /** \brief Contains combined output from cerr and cout. */
    QString combined_string_;
};
Q_DECLARE_METATYPE(ButtonProcessAdministrator)
Q_DECLARE_METATYPE(ButtonProcessAdministrator*)
#endif // BUTTONPROCESSADMINISTRATOR_H
