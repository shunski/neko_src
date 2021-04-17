#ifndef GENERICPARTS_H
#define GENERICPARTS_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>

template < typename Msg, typename CommandMsg, typename FeedbackMsg >
class GenericParts 
{
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

		CattyError check_msg_id( const Msg & ) const ;
		CattyError check_msg_id( const typename Msg::ConstPtr & ) const ;
		CattyError check_msg_id( const CommandMsg & ) const ;
		CattyError check_msg_id( const typename CommandMsg::ConstPtr & ) const ;
		CattyError check_msg_id( const FeedbackMsg & ) const ;
		CattyError check_msg_id( const typename FeedbackMsg::ConstPtr & ) const ;

		void print_msg_part_id_error( const Msg & ) const ;
		void print_msg_part_id_error( const typename Msg::ConstPtr & ) const ;
		void print_msg_part_id_error( const CommandMsg & ) const ;
		void print_msg_part_id_error( const typename CommandMsg::ConstPtr & ) const ;
		void print_msg_part_id_error( const FeedbackMsg & ) const ;
		void print_msg_part_id_error( const typename FeedbackMsg::ConstPtr & ) const ;

		void print_msg_id_error( const Msg & ) const ;
		void print_msg_id_error( const typename Msg::ConstPtr & ) const ;
		void print_msg_id_error( const CommandMsg & ) const ;
		void print_msg_id_error( const typename CommandMsg::ConstPtr & ) const ;
		void print_msg_id_error( const FeedbackMsg & ) const ;
		void print_msg_id_error( const typename FeedbackMsg::ConstPtr & ) const ;

	public:
		virtual CattyError set_msg( Msg & ) const =0;
		virtual CattyError set_CommandMsg( CommandMsg & ) const ;
		virtual CattyError set_FeedbackMsg( FeedbackMsg & ) const ;

		virtual CattyError set( const Msg & )=0;
		virtual CattyError set( const typename Msg::ConstPtr & )=0;
		virtual CattyError set( const CommandMsg & );
		virtual CattyError set( const typename CommandMsg::ConstPtr & );
		virtual CattyError set( const FeedbackMsg & );
		virtual CattyError set( const typename FeedbackMsg::ConstPtr & );

		virtual PartID get_part_id() const final ;
		virtual Uint8 get_id() const final ;

		virtual bool isValid() const final;
};


#endif
