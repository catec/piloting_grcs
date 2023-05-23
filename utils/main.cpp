#include <QsLog/QsLog.h>
#include <config.h>
#include <core/IAction.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QLoggingCategory>
#include <QThread>
#include <iostream>

#include "MainProgram.h"

using namespace gcs;
using namespace QsLogging;

static void qtConfig();
static void configureLogger();

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    qtConfig();

    QThread::currentThread()->setObjectName("Main");

    std::cout << "Starting up the application..." << std::endl;

    configureLogger();

    MainProgram mainProgram;
    int         ret = app.exec();

    std::cout << "Application shut down correctly" << std::endl;

    return ret;
}

static void qtConfig()
{
    QCoreApplication::setOrganizationName("FADA-CATEC");
    QCoreApplication::setApplicationName(
            QString("piloting_grcs_v%1.%2").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR));

    qRegisterMetaType<QSharedPointer<IDTO>>("QSharedPointer<IDTO>");
    qRegisterMetaType<ActionType>("ActionType");
}

static void configureLogger()
{
    QDir loggerDirectory(QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/logger/").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR)));

    if (!loggerDirectory.exists() && !loggerDirectory.mkpath(loggerDirectory.absolutePath())) {
        std::cerr << "Error: Not logging!!" << std::endl;
        return;
    }

    QDateTime currentDate = QDateTime::currentDateTime();

    const QFileInfoList loggersList = loggerDirectory.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);
    for (const auto& logger : loggersList) {
        if (logger.lastModified().daysTo(currentDate) > 30) {
            QFile::remove(logger.filePath());
        }
    }

    const QString loggerFileName
            = loggerDirectory.absoluteFilePath(QString("%1.log").arg(currentDate.toString("yyyy_MM_dd-hh_mm_ss")));
    std::cout << "Creating logger file: " << loggerFileName.toStdString() << std::endl;

    DestinationPtr fileDestination(DestinationFactory::MakeFileDestination(loggerFileName));
    DestinationPtr consoleDestination(DestinationFactory::MakeDebugOutputDestination());

    Logger& logger = Logger::instance();
    logger.setLoggingLevel(QsLogging::InfoLevel);
    logger.addDestination(fileDestination);
    logger.addDestination(consoleDestination);
}
