#ifndef PARTS_H
#define PARTS_H

#include<ros/ros.h>
#include<support_lib/Utilities.h>

template
<
	typename Msg,
	typename CommandMsg,
	typename FeedbackMsg
>
class Parts(){
	protected:
		const PartID part_id;
		const Uint8 id;
		bool valid;                         // will be set to false having had a serious error
		bool well_defined;                  // will be set to true after enough information filled out (as a particular 'state')
		ObjectState state;
		CommandParameters command_data;
		FeedbackParameters feedback_data;

		virtual void print() const =0;
		void print_id_error( Uint8 pID, Uint8 ID) const ;


		Parts( PartID, Uint8 id, objectState);
		Parts( Msg & ID );
		CattyPartsError check_msg_id( Msg & ) const ;
		CattyPartsError check_msg_id( typename Msg::ConstPtr & ) const ;
		CattyPartsError check_msg_id( CommandMsg & ) const ;
		CattyPartsError check_msg_id( typename CommandMsg::ConstPtr & ) const ;
		CattyPartsError check_msg_id( typename FeedbackMsg::ConstPtr & ) const ;

	public:
		virtual CattyPartsError set( Msg & )=0;
		virtual CattyPartsError set( typename Msg::ConstPtr & )=0;
		virtual CattyPartsError set( CommandMsg & );
		virtual CattyPartsError set( typename CommandMsg::ConstPtr & );
		virtual CattyPartsError set( FeedbackMsg & );
		virtual CattyPartsError set( typename FeedbackMsg::ConstPtr & );

		virtual CattyPartsError set_msg( Msg & ) const =0;
		virtual CattyPartsError set_CommandMsg( CommandMsg & ) const ;
		virtual CattyPartsError set_FeedbackMsg( FeedbackMsg & ) const ;
};


#endif
