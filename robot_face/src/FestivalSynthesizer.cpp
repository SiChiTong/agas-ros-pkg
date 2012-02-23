/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#include <pulse/error.h>
#include <pulse/gccmacro.h>
#include <pulse/simple.h>

#include <string>

#include "Config.h"
#include "FestivalSynthesizer.h"

FestivalSynthesizer::FestivalSynthesizer() :
    festival_initialized_(false),
    synth_speech_(false),
    punctuation_characters_(0)
{
    if (!festival_initialized_)
    {
        initFestival();
        festival_initialized_ = true;
    }

    smileys_.push_back(">:");
    smileys_.push_back(":)");
    smileys_.push_back(":(");
    smileys_.push_back(":O");
    smileys_.push_back(":o");
    smileys_.push_back(":!");
    smileys_.push_back(":&");

    punctuation_characters_.push_back(":");
    punctuation_characters_.push_back(")");
    punctuation_characters_.push_back("(");
    punctuation_characters_.push_back(".");
    punctuation_characters_.push_back("'");
    punctuation_characters_.push_back("\"");
    punctuation_characters_.push_back("}");
    punctuation_characters_.push_back("{");
    punctuation_characters_.push_back("§");
    punctuation_characters_.push_back("!");
    punctuation_characters_.push_back("?");
    punctuation_characters_.push_back("`");
    punctuation_characters_.push_back("´");
    punctuation_characters_.push_back("-");
    punctuation_characters_.push_back(";");
    punctuation_characters_.push_back(",");
    punctuation_characters_.push_back("€");
    punctuation_characters_.push_back("°");
    punctuation_characters_.push_back("<");
    punctuation_characters_.push_back(">");
    punctuation_characters_.push_back(".");
}

FestivalSynthesizer::~FestivalSynthesizer()
{
    festival_tidy_up();
}

void FestivalSynthesizer::initFestival()
{
    int heap_size = 210000;   // default scheme heap size
    int load_init_files = 1;  // we want the festival init files loaded

    festival_initialize(load_init_files, heap_size);

    try
    {
        const char* cfgFilename = "../config.cfg";

        Config cfg(cfgFilename);
        std::string voice = cfg.get("Voice");

        if ( voice == "female" )
        {
            festival_eval_command( "(voice_us1_mbrola)" );
            festival_eval_command( "(defvar Styles '((default 140 22 1.0)) \"Available voice styles\")" );
            festival_eval_command( "(defvar style_default 'default \"Default voice style\")" );
            festival_eval_command( "(defvar current_style 'none \"Current voice style\")" );
            festival_eval_command( "(define (Style selected_style) (let ((style (assoc selected_style Styles)))"
                                   "(if (not style) (set! style (assoc 'default Styles)))"
                                   "(let ((model_mean (cadr (assoc 'model_f0_mean  int_lr_params)))"
                                   "(model_std  (cadr (assoc 'model_f0_std   int_lr_params)))"
                                   "(new_mean (cadr style)) (new_std (cadr (cdr style)))"
                                   "(new_stretch (cadr (cdr (cdr style)))))"
                                   "(set! int_lr_params (list (list 'target_f0_mean new_mean)"
                                   "(list 'target_f0_std  new_std) (list 'model_f0_mean  model_mean)"
                                   "(list 'model_f0_std   model_std)))"
                                   "(Parameter.set 'Duration_Stretch new_stretch)"
                                   "(set! current_style (car style)) (list (car style) new_mean new_std new_stretch) ) ) )" );
            festival_eval_command( "(define (NewStyle style_name mean std stretch)"
                                   "(set! Styles (cons (list style_name mean std stretch) Styles)))" );
            festival_eval_command( "(NewStyle 'lisa 280 50 1.2)" );
            festival_eval_command( "(Style 'lisa)" );
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    // Check sampling rate
    EST_Wave wave;
    festival_text_to_wave( "init", wave );

    // Init pulse audio
    pa_sample_spec sample_spec;
    sample_spec.format = PA_SAMPLE_S16NE;
    sample_spec.channels = 1;
    sample_spec.rate = wave.sample_rate();

    pa_simple_ = pa_simple_new( NULL,                                   // Use the default server.
                                "FestivalSynthesizer",                  // Our application's name.
                                PA_STREAM_PLAYBACK,                     // Playback stream.
                                NULL,                                   // Use the default device.
                                "Festival Synthesizer (TalkingHead)",   // Description of our stream.
                                &sample_spec,                           // Our sample format.
                                NULL,                                   // Use default channel map
                                NULL,                                   // Use default buffering attributes.
                                NULL                                    // Ignore error code.
                                );

    if ( !pa_simple_ )
    {
        ROS_ERROR ( "Error initializing PulseAudio!" );
    }
}

void FestivalSynthesizer::run()
{
    ros::NodeHandle node_handle;
    subscriber_ = node_handle.subscribe( "robot_face/text_out", 1, &FestivalSynthesizer::callbackSynth, this );

    // Subscribe to ROS message
    talking_finished_publisher_ = node_handle.advertise<std_msgs::String>( "robot_face/talking_finished", 1 );

    while( ros::ok() )
    {
       ros::getGlobalCallbackQueue()->callOne( ros::WallDuration( 3 ) );

        if( synth_speech_ )
        {
            synthSpeech( text_for_synth_ );
            festival_wait_for_spooler();

            std_msgs::String msg;

            std::stringstream string_stream;
            string_stream << "talking_finished";
            msg.data = string_stream.str();

            talking_finished_publisher_.publish( msg );

            synth_speech_ = false;
        }
    }
}

void FestivalSynthesizer::callbackSynth( const std_msgs::String::ConstPtr& msg )
{
    text_for_synth_ = prepareText( msg->data );
    if( text_for_synth_.length() > 0 )
    {
        synth_speech_ = true;
    }
}

void FestivalSynthesizer::synthSpeech ( std::string text )
{
    if ( pa_simple_ )
    {
        EST_Wave wave;
        festival_text_to_wave( text.c_str(), wave );
        int error;
        pa_simple_write( pa_simple_, &(wave.a(0)), wave.length()*2, &error );
    }
    else
    {
        ROS_ERROR( "Cannot snyth speec. Initilization failed." );
    }
}

std::string FestivalSynthesizer::prepareText( std::string text )
{
    std::string tmp_text = text;

    tmp_text = trimSpaces( tmp_text );
    tmp_text = clearSmileys( tmp_text );

    size_t i_symbol = std::string::npos;

    for( unsigned int j = 0; j < punctuation_characters_.size(); j++ )
    {
    for( unsigned int i = 0; i < tmp_text.length(); i++ )
    {
            i_symbol = tmp_text.find( punctuation_characters_.at( j ), 0 );
            if( i_symbol != std::string::npos )
            {
                tmp_text.erase(i_symbol, punctuation_characters_.at( j ).length());
            }
        }
    }
    tmp_text = trimSpaces( tmp_text );

    return tmp_text;
}

std::string FestivalSynthesizer::clearSmileys( std::string text )
{
    std::string tmp_text = text;

    size_t i_smiley = std::string::npos;

    for( unsigned int j = 0; j < smileys_.size(); j++ )
    {
    for( unsigned int i = 0; i < tmp_text.length(); i++ )
    {
            i_smiley = tmp_text.find( smileys_.at( j ), 0 );
            if( i_smiley != std::string::npos )
            {
                tmp_text.erase(i_smiley, smileys_.at( j ).length());
            }
        }
    }

    return tmp_text;
}

std::string FestivalSynthesizer::trimSpaces( std::string text )
{
    std::string tmp_text = text;

    size_t startpos = tmp_text.find_first_not_of(" \t");
    size_t endpos = tmp_text.find_last_not_of(" \t");

    if(( std::string::npos == startpos ) || ( std::string::npos == endpos))
    {
        tmp_text = "";
    }
    else
    {
        tmp_text = tmp_text.substr( startpos, endpos-startpos+1 );
    }

    return tmp_text;
}

int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "FestivalSynthesizer");
    FestivalSynthesizer fs;
    fs.run();

    return 0;
}
