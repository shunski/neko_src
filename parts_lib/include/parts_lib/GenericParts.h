#ifndef GENERICPARTS_H
#define GENERICPARTS_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>

template < typename Msg, typename CommandMsg, typename FeedbackMsg >
class GenericParts(){
	protected:
		std::string child_name;             // Name of the child class (Name of parts)
		const PartID part_id;
		const Uint8 id;
		bool valid;                         // will be set to false having had a serious error
		bool well_defined;                  // will be set to true after enough information filled out (as a particular 'state')
		ObjectState state;

		virtual void print() const =0;
		void print_msg_id_error( Uint8 pID, Uint8 ID) const ;


		GenericParts( std::string ChildName, PartID, Uint8 id );
		GenericParts( Msg & ID );

		CattyPartsError check_msg_id( Msg & ) const final ;
		CattyPartsError check_msg_id( typename Msg::ConstPtr & ) const final ;
		CattyPartsError check_msg_id( CommandMsg & ) const final ;
		CattyPartsError check_msg_id( typename CommandMsg::ConstPtr & ) const final ;
		CattyPartsError check_msg_id( FeedbackMsg & ) const final ;
		CattyPartsError check_msg_id( typename FeedbackMsg::ConstPtr & ) const final ;

		void print_msg_part_id_error( Msg & ) const final ;
		void print_msg_part_id_error( typename Mgs::ConstPtr & ) const final ;
		void print_msg_part_id_error( CommandMsg & ) const final ;
		void print_msg_part_id_error( typename CommandMsg::ConstPtr & ) const final ;
		void print_msg_part_id_error( FeedbackMsg & ) const final ;
		void print_msg_part_id_error( typename FeedbackMsg::ConstPtr & ) const final ;

		void print_msg_id_error( Msg & ) const final ;
		void print_msg_id_error( typename Mgs::ConstPtr & ) const final ;
		void print_msg_id_error( CommandMsg & ) const final ;
		void print_msg_id_error( typename CommandMsg::ConstPtr & ) const final ;
		void print_msg_id_error( FeedbackMsg & ) const final ;
		void print_msg_id_error( typename FeedbackMsg::ConstPtr & ) const final ;

	public:
		virtual CattyPartsError set( const Msg & )=0;
		virtual CattyPartsError set( const typename Msg::ConstPtr & )=0;
		virtual CattyPartsError set( const CommandMsg & );
		virtual CattyPartsError set( const typename CommandMsg::ConstPtr & );
		virtual CattyPartsError set( const FeedbackMsg & );
		virtual CattyPartsError set( const typename FeedbackMsg::ConstPtr & );

		PartID get_part_id() const final ;
		Uint8 get_id() const final ;

		bool isValid() const final;
};


#endif
