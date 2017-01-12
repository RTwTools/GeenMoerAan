/****************************************************************************
** Meta object code from reading C++ file 'function.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../Controller Source code/function.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'function.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Function[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // methods: signature, parameters, type, tag, flags
      14,   10,    9,    9, 0x02,
      48,   42,    9,    9, 0x02,
      70,   10,    9,    9, 0x02,
     100,    9,   92,    9, 0x02,
     115,    9,   92,    9, 0x02,

       0        // eod
};

static const char qt_meta_stringdata_Function[] = {
    "Function\0\0txt\0start_stop_command(QString)\0"
    "value\0detect_holes(QString)\0"
    "setWorkSpace(QString)\0QString\0"
    "getWorkspace()\0get_currentTime()\0"
};

void Function::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Function *_t = static_cast<Function *>(_o);
        switch (_id) {
        case 0: _t->start_stop_command((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->detect_holes((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->setWorkSpace((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: { QString _r = _t->getWorkspace();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 4: { QString _r = _t->get_currentTime();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Function::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Function::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Function,
      qt_meta_data_Function, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Function::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Function::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Function::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Function))
        return static_cast<void*>(const_cast< Function*>(this));
    return QObject::qt_metacast(_clname);
}

int Function::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
