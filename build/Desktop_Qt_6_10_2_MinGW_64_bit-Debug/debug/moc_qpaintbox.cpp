/****************************************************************************
** Meta object code from reading C++ file 'qpaintbox.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.10.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../qpaintbox.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qpaintbox.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.10.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN9QPaintBoxE_t {};
} // unnamed namespace

template <> constexpr inline auto QPaintBox::qt_create_metaobjectdata<qt_meta_tag_ZN9QPaintBoxE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "QPaintBox",
        "OnMousePress",
        "",
        "QMouseEvent*",
        "event",
        "OnMouseRelease",
        "OnMouseMove",
        "OnWheelMove",
        "QWheelEvent*"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'OnMousePress'
        QtMocHelpers::SignalData<void(QMouseEvent *)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 },
        }}),
        // Signal 'OnMouseRelease'
        QtMocHelpers::SignalData<void(QMouseEvent *)>(5, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 },
        }}),
        // Signal 'OnMouseMove'
        QtMocHelpers::SignalData<void(QMouseEvent *)>(6, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 },
        }}),
        // Signal 'OnWheelMove'
        QtMocHelpers::SignalData<void(QWheelEvent *)>(7, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 8, 4 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<QPaintBox, qt_meta_tag_ZN9QPaintBoxE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject QPaintBox::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9QPaintBoxE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9QPaintBoxE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN9QPaintBoxE_t>.metaTypes,
    nullptr
} };

void QPaintBox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<QPaintBox *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->OnMousePress((*reinterpret_cast<std::add_pointer_t<QMouseEvent*>>(_a[1]))); break;
        case 1: _t->OnMouseRelease((*reinterpret_cast<std::add_pointer_t<QMouseEvent*>>(_a[1]))); break;
        case 2: _t->OnMouseMove((*reinterpret_cast<std::add_pointer_t<QMouseEvent*>>(_a[1]))); break;
        case 3: _t->OnWheelMove((*reinterpret_cast<std::add_pointer_t<QWheelEvent*>>(_a[1]))); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (QPaintBox::*)(QMouseEvent * )>(_a, &QPaintBox::OnMousePress, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (QPaintBox::*)(QMouseEvent * )>(_a, &QPaintBox::OnMouseRelease, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (QPaintBox::*)(QMouseEvent * )>(_a, &QPaintBox::OnMouseMove, 2))
            return;
        if (QtMocHelpers::indexOfMethod<void (QPaintBox::*)(QWheelEvent * )>(_a, &QPaintBox::OnWheelMove, 3))
            return;
    }
}

const QMetaObject *QPaintBox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QPaintBox::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9QPaintBoxE_t>.strings))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int QPaintBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QPaintBox::OnMousePress(QMouseEvent * _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 0, nullptr, _t1);
}

// SIGNAL 1
void QPaintBox::OnMouseRelease(QMouseEvent * _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 1, nullptr, _t1);
}

// SIGNAL 2
void QPaintBox::OnMouseMove(QMouseEvent * _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 2, nullptr, _t1);
}

// SIGNAL 3
void QPaintBox::OnWheelMove(QWheelEvent * _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 3, nullptr, _t1);
}
QT_WARNING_POP
