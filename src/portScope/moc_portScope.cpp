/****************************************************************************
** DataPlot meta object code from reading C++ file 'data_plot.h'
**
** Created: Sat Jul 24 12:32:59 2010
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "portScope.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *DataPlot::className() const
{
    return "DataPlot";
}

QMetaObject *DataPlot::metaObj = 0;
static QMetaObjectCleanUp cleanUp_DataPlot( "DataPlot", &DataPlot::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString DataPlot::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DataPlot", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString DataPlot::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DataPlot", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* DataPlot::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QwtPlot::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "interval", &static_QUType_double, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"setTimerInterval", 1, param_slot_0 };
    static const QUMethod slot_1 = {"toggleAcquire", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "setTimerInterval(double)", &slot_0, QMetaData::Public },
	{ "toggleAcquire()", &slot_1, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"DataPlot", parentObject,
	slot_tbl, 2,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_DataPlot.setMetaObject( metaObj );
    return metaObj;
}

void* DataPlot::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "DataPlot" ) )
	return this;
    return QwtPlot::qt_cast( clname );
}

bool DataPlot::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: setTimerInterval((double)static_QUType_double.get(_o+1)); break;
    case 1: toggleAcquire(); break;
    default:
	return QwtPlot::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool DataPlot::qt_emit( int _id, QUObject* _o )
{
    return QwtPlot::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool DataPlot::qt_property( int id, int f, QVariant* v)
{
    return QwtPlot::qt_property( id, f, v);
}

bool DataPlot::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
