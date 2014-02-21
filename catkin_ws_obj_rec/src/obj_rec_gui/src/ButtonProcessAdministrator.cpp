#include "ButtonProcessAdministrator.h"
#include <iostream>
#include <QMessageBox>
#include <QMenu>
#include <QRegExp>

ButtonProcessAdministrator::ButtonProcessAdministrator(QString name, QString command, QStringList arguments, QObject *parent):
    QObject(parent),
    current_process_state_(NotRunning),
    name_(name),
    process_(new QProcess()),
    command_(command),
    arguments_(arguments)
{
    process_->setProcessChannelMode(QProcess::SeparateChannels);
    connectProcess();
}

ButtonProcessAdministrator::ButtonProcessAdministrator(QObject *parent):
    QObject(parent),
    current_process_state_(NotRunning),
    process_(new QProcess())
{
    process_->setProcessChannelMode(QProcess::SeparateChannels);
    connectProcess();
}

ButtonProcessAdministrator::ButtonProcessAdministrator(const ButtonProcessAdministrator &copy):
    QObject(),
    name_(copy.getName()),
    process_(new QProcess())
{
    buttons_ = copy.buttons_;
    process_->setProcessChannelMode(QProcess::SeparateChannels);
    this -> setCommand(copy.getCommand(), copy.getArguments());
    connectProcess();
}

ButtonProcessAdministrator::~ButtonProcessAdministrator()
{
    if(process_->state() == QProcess::Running){
        stopProcess();
    }
    delete process_;
}

QString ButtonProcessAdministrator::getName() const
{
    return name_;
}

QString ButtonProcessAdministrator::getCommand() const
{
    return command_;
}

QStringList ButtonProcessAdministrator::getArguments() const
{
    return arguments_;
}

void ButtonProcessAdministrator::addButton(QAbstractButton* button)
{
    buttons_.push_back(button);
    setButtonTextStart();
    connect(button,SIGNAL(clicked()),this,SLOT(handleClick()));
}

void ButtonProcessAdministrator::addButton(QPushButton *button)
{
    buttons_.push_back(button);
    setButtonTextStart();
    connect(button,SIGNAL(clicked()),this,SLOT(handleClick()));
}

void ButtonProcessAdministrator::addButton(RosProcessButton *button)
{
    buttons_.push_back(button);
    setButtonTextStart();
    connect(button, SIGNAL(clicked()), this, SLOT(handleClick()));
    connect(button, SIGNAL(killPushed()), this->process_, SLOT(kill()));
    connect(button, SIGNAL(startInTerminalPushed()), this, SLOT(startInExternalTerminal()));
}

QProcess::ProcessState ButtonProcessAdministrator::getState() const
{
    return this -> process_->state();
}

QString ButtonProcessAdministrator::getCombinedString() const
{
    return combined_string_;
}

QString ButtonProcessAdministrator::getStderrString() const
{
    return stderr_string_;
}

QString ButtonProcessAdministrator::getStdcoutString() const
{
    return stdcout_string_;
}

void ButtonProcessAdministrator::stopProcess()
{
    process_->terminate();
    current_process_state_ = ShuttingDown;
    setButtonTextShuttingDown();
}

void ButtonProcessAdministrator::startProcess()
{
    this -> combined_string_.append("--------------------------------------------------------------------------------------------\n");
    process_->start(command_, arguments_);
    current_process_state_ = Starting;
}

void ButtonProcessAdministrator::setName(QString name)
{
    this -> name_ = name;
}

void ButtonProcessAdministrator::setCommand(QString command, QStringList arguments)
{
    this->command_ = command;
    this->arguments_ = arguments;
}

//Modifies the text of all buttons connected with the Admin. You can't disconnect buttons so far!
void ButtonProcessAdministrator::setButtonText(QString text)
{
    for(int i = 0; i < buttons_.size(); ++i)
    {
        buttons_[i]->setText(text);
    }
}

void ButtonProcessAdministrator::connectProcess()
{
    connect(this -> process_, SIGNAL(readyReadStandardOutput()), this, SLOT(readStdoutProcess()));
    connect(this -> process_, SIGNAL(readyReadStandardError()), this, SLOT(readStderrProcess()));
    connect(this -> process_, SIGNAL(error(QProcess::ProcessError)), this, SLOT(handleProcessError(QProcess::ProcessError)));
    connect(this -> process_, SIGNAL(stateChanged(QProcess::ProcessState)), this, SLOT(handleProcessStateChange(QProcess::ProcessState)) );
}

//Handles a Click on any connected Button
void ButtonProcessAdministrator::handleClick()
{
    if(current_process_state_ == NotRunning)
    {
        startProcess();
    }
    else if(current_process_state_ == Running)
    {
        stopProcess();
    }
    else if(current_process_state_ == Starting ||
            current_process_state_ == ShuttingDown)
    {

    }
}

void ButtonProcessAdministrator::handleProcessError(QProcess::ProcessError error)
{
    switch(error)
    {
    case QProcess::FailedToStart:
        QMessageBox::information(0,"FailedToStart",this->name_ + " FailedToStart");
        break;
    case QProcess::Crashed:
        QMessageBox::information(0,"Crashed", this->name_ + " crashed");
        break;
    case QProcess::Timedout:
        QMessageBox::information(0,"FailedToStart",this->name_ + " FailedToStart");
        break;
    case QProcess::WriteError:
        QMessageBox::information(0,"Timedout",this->name_ + " Timedout");
        break;
    case QProcess::ReadError:
        QMessageBox::information(0,"ReadError",this->name_ + " ReadError");
        break;
    case QProcess::UnknownError:
        QMessageBox::information(0,"UnknownError",this->name_ + " UnknownError");
        break;
    default:
        QMessageBox::information(0,"default",this->name_ + " default");
        break;
    }
}

void ButtonProcessAdministrator::startInExternalTerminal()
{
    QString processCommand = command_ + " " + arguments_.join(" ");
    //TODO: get Default terminal of OS
    QString terminalCommand = "gnome-terminal -e \"bash -c '" + processCommand + "; bash'\"";

    this->process_->start(terminalCommand);
}

void ButtonProcessAdministrator::handleProcessStateChange(QProcess::ProcessState state)
{
    switch(state)
    {
    case QProcess::Starting:
        setButtonTextStarting();
        current_process_state_ = Starting;
    break;
    case QProcess::NotRunning:
        setButtonTextStart();
        current_process_state_ = NotRunning;
        break;
    case QProcess::Running:
        setButtonTextStop();
        current_process_state_ = Running;
        break;
    }
}

void ButtonProcessAdministrator::readStdoutProcess()
{
    QString coutString(this -> process_->readAllStandardOutput());

  // Replace the ansi codes
    coutString.remove(QRegExp ("\e\\[[0-9]{0,3}m"));
    coutString.remove(QRegExp ("\e\\][0-9];"));

    coutString.remove(QRegExp ("\a"));

    this -> stdcout_string_.append(coutString);
    this -> combined_string_.append(coutString);

    emit stdcoutStringChanged(this -> stdcout_string_);
    emit stdcoutStringChanged();
    emit combinedStringChanged(this -> combined_string_);
    emit combinedStringChanged();
}

void ButtonProcessAdministrator::readStderrProcess()
{
    QString errString(this -> process_->readAllStandardError());

  // Replace the ansi codes
    errString.remove(QRegExp ("\e\\[[0-9]{0,3}m"));
    errString.remove(QRegExp ("\e\\][0-9];"));
    errString.remove(QRegExp ("\a"));

    this -> stderr_string_.append(errString);
    this -> combined_string_.append(errString);

    emit stderrStringChanged(this -> stderr_string_);
    emit stderrStringChanged();
    emit combinedStringChanged(this -> combined_string_);
    emit combinedStringChanged();
}

void ButtonProcessAdministrator::setButtonTextStart()
{
    setButtonText(QString("Start ").append(this->name_));
}

void ButtonProcessAdministrator::setButtonTextStop()
{
    setButtonText(QString("Stop ").append(this->name_));
}

void ButtonProcessAdministrator::setButtonTextStarting()
{
    setButtonText(QString("Starting ").append(this->name_));
}

void ButtonProcessAdministrator::setButtonTextShuttingDown()
{
    setButtonText(QString("Stopping ").append(this->name_));
}
