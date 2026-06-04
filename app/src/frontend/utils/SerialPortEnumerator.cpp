#include "frontend/utils/SerialPortEnumerator.hpp"

#include <algorithm>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace
{
int comPortNumber(const QString &portName)
{
    if (!portName.startsWith(QStringLiteral("COM"), Qt::CaseInsensitive))
        return 0;

    bool ok = false;
    const int number = portName.mid(3).toInt(&ok);
    return ok ? number : 0;
}
} // namespace

namespace ui
{
QStringList enumerateSerialPortNames()
{
    QStringList ports;

    HKEY key = nullptr;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                      L"HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0,
                      KEY_READ,
                      &key) != ERROR_SUCCESS)
        return ports;

    DWORD index = 0;
    for (;; ++index)
    {
        wchar_t valueName[256] = {};
        wchar_t data[256] = {};
        DWORD valueNameChars = static_cast<DWORD>(std::size(valueName));
        DWORD dataBytes = static_cast<DWORD>(sizeof(data));
        DWORD valueType = 0;

        const LONG result = RegEnumValueW(key,
                                          index,
                                          valueName,
                                          &valueNameChars,
                                          nullptr,
                                          &valueType,
                                          reinterpret_cast<LPBYTE>(data),
                                          &dataBytes);
        if (result == ERROR_NO_MORE_ITEMS)
            break;
        if (result != ERROR_SUCCESS || valueType != REG_SZ)
            continue;

        const QString portName = QString::fromWCharArray(data).trimmed();
        if (!portName.isEmpty() && !ports.contains(portName, Qt::CaseInsensitive))
            ports.append(portName);
    }

    RegCloseKey(key);

    std::sort(ports.begin(), ports.end(), [](const QString &left, const QString &right) {
        const int leftNumber = comPortNumber(left);
        const int rightNumber = comPortNumber(right);
        if (leftNumber != 0 && rightNumber != 0 && leftNumber != rightNumber)
            return leftNumber < rightNumber;
        return left.compare(right, Qt::CaseInsensitive) < 0;
    });

    return ports;
}
} // namespace ui
