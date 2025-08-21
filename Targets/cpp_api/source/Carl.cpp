#include "Carl.h"

#include <carl/Carl.h>

#include <arcana/threading/task.h>

namespace carl::cpp
{
    struct Recognizer::Impl
    {
        Impl(carl::Session& session, action::Definition& definition)
            : Recognizer{session, definition}
        {
        }

        action::Recognizer Recognizer;
    };

    struct Session::Impl
    {
        Impl(bool isSingleThreaded)
            : Session{ isSingleThreaded }
        {
        }

        carl::Session Session;
    };

    Recognizer::Recognizer(std::unique_ptr<Impl> impl)
        : m_impl{ std::move(impl) }
    {
    }

    Recognizer::~Recognizer()
    {
    }

    double Recognizer::currentScore() const
    {
        return m_impl->Recognizer.currentScore();
    }

    Session::Session(bool isSingleThreaded)
        : m_impl{ std::make_unique<Session::Impl>(isSingleThreaded) }
    {
    }

    Session::~Session()
    {
    }

    std::future<std::unique_ptr<Recognizer>> Session::createRecognizerFromSerializedDefinitionAsync(uint8_t* bytes, size_t bytesLength)
    {
        auto promise = std::make_unique<std::promise<std::unique_ptr<Recognizer>>>();
        auto future = promise->get_future();
        carl::Deserialization deserialization{ bytes };
        auto definition{ std::make_unique<carl::action::Definition>(deserialization) };
        arcana::make_task(m_impl->Session.processingScheduler(), arcana::cancellation::none(), [&session = m_impl->Session, definitionPtr = std::move(definition), promise = std::move(promise)]() {
            auto recognizerPtr = new Recognizer(std::make_unique<Recognizer::Impl>(session, *definitionPtr));
            promise->set_value(std::unique_ptr<Recognizer>{recognizerPtr});
        });
        return future;
    }

    void Session::AddInputSample(InputSample cppSample)
    {
        carl::InputSample sample{};
        sample.Timestamp = cppSample.Timestamp;
        // TODO: Recharacterize the cpp input sample as a regular input sample.
        // ...
        m_impl->Session.addInput(sample);
    }
}