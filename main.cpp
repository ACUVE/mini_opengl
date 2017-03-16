#define GLEW_STATIC
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>
#include <iterator>
#include <stdexcept>
#include <cstdio>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#if defined( _MSC_VER ) && !defined( _M_AMD64 )
#error "only for x64"
#endif

#ifdef _MSC_VER
#pragma comment( lib, "Release\\x64\\glew32s.lib" )
#pragma comment( lib, "glfw3.lib" )
#pragma comment( lib, "opengl32" )
#endif

namespace model
{
	// plyを適当にパースする奴。全然正しくない。
	static
	void load_ply( std::string const &filename, std::vector< float > &point, std::vector< unsigned int > &index )
	{
		point.clear();
		index.clear();
		std::ifstream ifs( filename, std::ios::binary );
		if( !ifs.is_open() ) throw std::runtime_error( "load_ply: cannnot open " + filename );
		int vertexline = 0, faceline = 0;
		enum class format{
			ASCII, BINARY_LE, UNKNOWN
		} read_format = format::UNKNOWN;
		std::string line;
		while( std::getline( ifs, line ) )
		{
			if( line.substr( 0, 7 ) == "format " )
			{
				if( line.substr( 7, 6 ) == "ascii " )
				{
					read_format = format::ASCII;
				}
				else if( line.substr( 7, 21 ) == "binary_little_endian " )
				{
					read_format = format::BINARY_LE;
				}
			}
			else if( line.substr( 0, 15 ) == "element vertex " )
			{
				vertexline = std::atoi( line.substr( 15 ).c_str() );
			}
			else if( line.substr( 0, 13 ) == "element face " )
			{
				faceline = std::atoi( line.substr( 13 ).c_str() );
			}
			else if( line.substr( 0, 10 ) == "end_header" )
			{
				break;
			}
		}
		if( read_format == format::UNKNOWN ) throw std::runtime_error( "load_ply: cannot recognize ply format" );
		if( vertexline == 0 || faceline == 0 ) throw std::runtime_error( "load_ply: oh no" );
		point.resize( vertexline * 3 );
		index.resize( faceline * 3 );
		switch( read_format )
		{
		case format::ASCII:
			{
				for( int i = 0; i < vertexline && std::getline( ifs, line ); ++i )
				{
					std::istringstream iss( line );
					auto const ind = i * 3;
					iss >> point[ ind + 0 ] >> point[ ind + 1 ] >> point[ ind + 2 ];
				}
				for( int i = 0; i < faceline && std::getline( ifs, line ); ++i )
				{
					std::istringstream iss( line );
					auto const ind = i * 3;
					int dummy;
					iss >> dummy >> index[ ind + 0 ] >> index[ ind + 1 ] >> index[ ind + 2 ];
					if( dummy != 3 )
					{
						point.clear();
						index.clear();
						return;
					}
				}
			}
			break;
		case format::BINARY_LE:
			{
				for( int i = 0; i < vertexline; ++i )
				{
					auto const ind = i * 3;
					for( int j = 0; j < 3; ++j ) ifs.read( reinterpret_cast< char * >( &point[ ind + j ] ), sizeof( float ) );
				}
				for( int i = 0; i < faceline; ++i )
				{
					auto const ind = i * 3;
					char dummy;
					ifs.read( &dummy, sizeof( char ) );
					if( dummy != 3 )
					{
						point.clear();
						index.clear();
						return;
					}
					for( int j = 0; j < 3; ++j ) ifs.read( reinterpret_cast< char * >( &index[ ind + j ] ), sizeof( float ) );
				}
			}
			break;
		case format::UNKNOWN:
			throw std::logic_error( "never comes here" );
		}
	}
	static
	std::tuple< std::vector< float >, std::vector< unsigned int > > load_ply( std::string const &filename )
	{
		std::vector< float > vf;
		std::vector< unsigned int > uf;
		load_ply( filename, vf, uf );
		return std::make_tuple( std::move( vf ), std::move( uf ) );
	}
	// 各頂点での法線ベクトルの計算
	static
	void calc_normal( std::vector< float > const &point, std::vector< unsigned int > const &index, std::vector< float > &normal )
	{
		auto const num_of_point = std::size( point ) / 3;
		normal.clear();
		normal.reserve( num_of_point * 3 );
		std::vector< bool > flag( num_of_point, false );
		std::vector< glm::vec3 > tmp_normal( num_of_point, glm::vec3( 0.0f, 0.0f, 0.0f ) );
		auto const index_size = std::size( index );
		for( auto i = 0u; i + 2 < index_size; i += 3 )
		{
			auto const *pp1 = &point[ index[ i + 0 ] * 3 ], *pp2 = &point[ index[ i + 1 ] * 3 ], *pp3 = &point[ index[ i + 2 ] * 3 ];
			glm::vec3 const p1( pp1[ 0 ], pp1[ 1 ], pp1[ 2 ] ), p2( pp2[ 0 ], pp2[ 1 ], pp2[ 2 ]  ), p3( pp3[ 0 ], pp3[ 1 ], pp3[ 2 ] );
			auto const n = glm::normalize( glm::cross( p1 - p2, p1 - p3 ) );
			for( auto i : { index[ i + 0 ], index[ i + 1 ], index[ i + 2 ] } )
			{
				flag[ i ] = true;
				tmp_normal[ i ] += n;
			}
		}
		for( auto i = 0u; i < num_of_point; ++i )
		{
			auto const n = flag[ i ] ? glm::normalize( tmp_normal[ i ] ) : glm::vec3( 0.0f, 0.0f, 0.0f );
			normal.insert( normal.end(), { n.x, n.y, n.z });
		}
	}
	static
	std::vector< float > calc_normal( std::vector< float > const &point, std::vector< unsigned int > const &index )
	{
		std::vector< float > normal;
		calc_normal( point, index, normal );
		return std::move( normal );
	}
	template< typename VecFloat, typename Func >
	static
	typename std::enable_if<
		std::is_same<
			typename std::remove_cv< VecFloat >::type,
			std::vector< float >
		>::value
	>::type foreach_point( VecFloat &point, Func func )
	{
		auto const size = std::size( point );
		for( auto i = 0u; i + 2u < size; i += 3 )
		{
			func( point[ i + 0u ], point[ i + 1u ], point[ i + 2u ] );
		}
	}
	static
	void minmax_coord( std::vector< float > const &point, std::tuple< float, float > &x_minmax, std::tuple< float, float > &y_minmax, std::tuple< float, float > &z_minmax )
	{
		x_minmax = y_minmax = z_minmax = std::make_tuple( std::numeric_limits< float >::infinity(), -std::numeric_limits< float >::infinity() );
		float &xmin = std::get< 0 >( x_minmax ), &xmax = std::get< 1 >( x_minmax ), &ymin = std::get< 0 >( y_minmax ), &ymax = std::get< 1 >( y_minmax ), &zmin = std::get< 0 >( z_minmax ), &zmax = std::get< 1 >( z_minmax );
		foreach_point(
			point,
			[ & ]( float x, float y, float z ){
				if( x < xmin ) xmin = x;
				if( xmax < x ) xmax = x;
				if( y < ymin ) ymin = y;
				if( ymax < y ) ymax = y;
				if( z < zmin ) zmin = z;
				if( zmax < z ) zmax = z;
			}
		);
	}
	static
	std::tuple< std::tuple< float, float >, std::tuple< float, float >, std::tuple< float, float > > minmax_coord( std::vector< float > const &point )
	{
		std::tuple< float, float > x, y, z;
		minmax_coord( point, x, y, z );
		return std::make_tuple( x, y, z );
	}
}
namespace opengl
{
	GLuint compile_shader( char const *vertex_shader_src, char const *fragment_shader_src )
	{
		GLint result, log_length;
		std::vector< char > log_buff;

		GLuint vertex_shader = glCreateShader( GL_VERTEX_SHADER );
		char const *pvss = vertex_shader_src;
		glShaderSource( vertex_shader, 1, &pvss, nullptr );
		glCompileShader( vertex_shader );
		glGetShaderiv( vertex_shader, GL_COMPILE_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetShaderiv( vertex_shader, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetShaderInfoLog( vertex_shader, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << "Vertex Shader Compile Log\n";
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}

		GLuint fragment_shader = glCreateShader( GL_FRAGMENT_SHADER );
		char const *pfss = fragment_shader_src;
		glShaderSource( fragment_shader, 1, &pfss, nullptr );
		glCompileShader( fragment_shader );
		glGetShaderiv( fragment_shader, GL_COMPILE_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetShaderiv( fragment_shader, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetShaderInfoLog( fragment_shader, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << "Fragment Shader Compile Log\n";
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}

		GLuint program = glCreateProgram();
		glAttachShader( program, vertex_shader );
		glAttachShader( program, fragment_shader );
		glLinkProgram( program );
		glGetProgramiv( program, GL_LINK_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetProgramiv( program, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetProgramInfoLog( program, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << "Shader Link Log\n";
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}
		glDeleteShader( vertex_shader );
		glDeleteShader( fragment_shader );

		return program;
	}
	template< typename T, typename Alloc >
	GLuint make_gl_buffer( GLenum const type, GLenum const usage, std::vector< T, Alloc > const &vec )
	{
		GLuint id;
		glGenBuffers( 1, &id );
		glBindBuffer( type, id );
		auto const size = std::size( vec );
		glBufferData( type, sizeof( vec[ 0 ] ) * size, size ? &vec[ 0 ] : nullptr, usage );
		return id;
	}
}

template< typename T >
auto defer( T pfunc ) noexcept
{
	class Call
	{
	private:
		T pfunc;
	public:
		Call( T _pfunc ) noexcept : pfunc( _pfunc ){}
		Call( Call const & ) = delete;
		Call( Call &&r ) noexcept : pfunc( r.pfunc ){ r.pfunc = nullptr; }
		Call &operator=( Call const & ) = delete;
		Call &operator=( Call && ) = delete;
		~Call() noexcept{ if( pfunc != nullptr ) pfunc(); }
	};
	return Call{ pfunc };
}
std::string readallfile( std::string const &filename )
{
	std::ifstream ifs( filename );
	if( !ifs.is_open() ) throw std::runtime_error( "readallfile: cannot open file " + filename );
	std::string tmp;
	if( !std::getline( ifs, tmp, '\0' ) ) return {};
	return tmp;
}

constexpr auto WIDTH = 640u, HEIGHT = 480u;

struct window_data
{
	glm::mat4 proj, view, model;
	bool draged{ false };
	float xpos, ypos;
	GLuint program;
};

void window_framebuffer_size_callback( GLFWwindow *window, int width, int height )
{
	auto data = static_cast< window_data * >( glfwGetWindowUserPointer( window ) );
	if( !data ) return;
	data->proj = glm::perspective( glm::radians( 30.0f ), static_cast< float >( width ) / height, 1.0f, 1000.0f );
	glfwMakeContextCurrent( window );
	glViewport( 0, 0, width, height );
}
void window_cursor_pos_callback( GLFWwindow *window, double _xpos, double _ypos )
{
	auto data = static_cast< window_data * >( glfwGetWindowUserPointer( window ) );
	if( !data ) return;
	auto xpos = static_cast< float >( _xpos ), ypos = static_cast< float >( _ypos );
	if( data->draged )
	{
	    auto const dx = (data->xpos - xpos) / 100.0f, dy = (data->ypos - ypos) / 100.0f;
	    auto const h = std::hypot( dx, dy );
	    data->model = glm::rotate( h, glm::vec3( -dy, dx, 0.0f ) ) * data->model;
	}
	data->xpos = xpos; data->ypos = ypos;
}
void window_mouse_button_callback( GLFWwindow *window, int button, int action, int mods )
{
	auto data = static_cast< window_data * >( glfwGetWindowUserPointer( window ) );
	if( !data ) return;
	switch( button )
	{
	case GLFW_MOUSE_BUTTON_LEFT:
	    switch( action ){
	    case GLFW_PRESS: data->draged = true; break;
	    case GLFW_RELEASE: data->draged = false; break;
	    }
	    break;
	}
}
void window_key_callback( GLFWwindow *window, int key, int scancode, int action, int mods )
{
	auto data = static_cast< window_data * >( glfwGetWindowUserPointer( window ) );
	if( !data ) return;
	switch( key )
	{
	case GLFW_KEY_R:
	    glfwMakeContextCurrent( window );
		try
		{
			auto const vs = readallfile( "vertexshader.txt" );
			auto const fs = readallfile( "fragmentshader.txt" );
			data->program = opengl::compile_shader( vs.c_str(), fs.c_str() );
		}
		catch( ... )
		{}
	}
}

int main( int argc, char **argv )
try
{
	auto const ply = model::load_ply( "stanford_bunny.ply" );
	auto const &point = std::get< 0 >( ply );
	auto const &index = std::get< 1 >( ply );
	auto const normal = model::calc_normal( point, index );
	
	if( !glfwInit() ) throw std::runtime_error( "glfwInit error" );
	auto ac1 = defer( &glfwTerminate );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR , 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR , 3 );
	auto main_window = glfwCreateWindow( WIDTH, HEIGHT, "ply view", nullptr, nullptr );
	if( !main_window ) throw std::runtime_error( "glfwCreateWindow error" );
	glfwMakeContextCurrent( main_window );
	glewExperimental = GL_TRUE;
	if( glewInit() != GLEW_OK ) throw std::runtime_error( "glewInit error" );

	window_data main_window_data;
	glfwSetWindowUserPointer( main_window, &main_window_data );

	glEnable( GL_DEPTH_TEST );
	GLuint vao;
	glGenVertexArrays( 1, &vao );
	glBindVertexArray( vao );
	
	auto const point_buffer = opengl::make_gl_buffer( GL_ARRAY_BUFFER, GL_STATIC_DRAW, point );
	auto const index_buffer = opengl::make_gl_buffer( GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW, index );
	auto const normal_buffer = opengl::make_gl_buffer( GL_ARRAY_BUFFER, GL_STATIC_DRAW, normal );

	auto const minmax_coord = model::minmax_coord( point );
	auto const &xminmax = std::get< 0 >( minmax_coord ), &yminmax = std::get< 1 >( minmax_coord ), &zminmax = std::get< 2 >( minmax_coord );
	auto const &xmin = std::get< 0 >( xminmax ), &xmax = std::get< 1 >( xminmax );
	auto const &ymin = std::get< 0 >( yminmax ), &ymax = std::get< 1 >( yminmax );
	auto const &zmin = std::get< 0 >( zminmax ), &zmax = std::get< 1 >( zminmax );
	auto const lx = (xmin + xmax) / 2.0f, ly = (ymin + ymax) / 2.0f, lz = (zmin + zmax) / 2.0f;
	auto const xl = xmax - xmin, yl = ymax - ymin, zl = zmax - zmin;
	
	main_window_data.proj = glm::perspective( glm::radians( 30.0f ), static_cast< float >( WIDTH ) / HEIGHT, 0.01f, 10000.0f );
	main_window_data.view = glm::lookAt( glm::vec3( 0.0f, 0.0f, -zl * 3 ), glm::vec3( 0.0f, 0.0f, 0.0f ), glm::vec3( 0.0f, -1.0f, 0.0f ) );
	main_window_data.model = glm::mat4( 1.0f );
	glm::mat4 const c_model = glm::translate( -glm::vec3( lx, ly, lz ) );

	glViewport( 0, 0, WIDTH, HEIGHT );

	glfwSetFramebufferSizeCallback( main_window, window_framebuffer_size_callback );
	glfwSetCursorPosCallback( main_window, window_cursor_pos_callback );
	glfwSetMouseButtonCallback( main_window, window_mouse_button_callback );
	glfwSetKeyCallback( main_window, window_key_callback );
	
	{
		try
		{
			auto const vs = readallfile( "vertexshader.txt" );
			auto const fs = readallfile( "fragmentshader.txt" );
			main_window_data.program = opengl::compile_shader( vs.c_str(), fs.c_str() );
		}
		catch( ... )
		{}
	}
	
	while( !glfwWindowShouldClose( main_window ) )
	{
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		
		glm::mat4 const model = main_window_data.model * c_model;
		glm::mat4 const mvp = main_window_data.proj * main_window_data.view * model;
		glm::mat3 const Rmat( model );
		
		glUseProgram( main_window_data.program );
		glUniformMatrix4fv( glGetUniformLocation( main_window_data.program, "Hmat" ), 1, GL_FALSE, &mvp[ 0 ][ 0 ] );
		glUniformMatrix3fv( glGetUniformLocation( main_window_data.program, "Rmat" ), 1, GL_FALSE, &Rmat[ 0 ][ 0 ] );
		glUniform3f( glGetUniformLocation( main_window_data.program, "Lvec" ), 0.577f, 0.577f, 0.577f );
		
		glEnableClientState( GL_VERTEX_ARRAY );
		
		glEnableVertexAttribArray( 0 );
		glBindBuffer( GL_ARRAY_BUFFER, point_buffer );
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast< void * >( 0 ) );
		
		glEnableVertexAttribArray( 1 );
		glBindBuffer( GL_ARRAY_BUFFER, normal_buffer );
		glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast< void * >( 0 ) );
		
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, index_buffer );
		glDrawElements( GL_TRIANGLES, static_cast< GLsizei >( index.size() ), GL_UNSIGNED_INT, reinterpret_cast< void * >( 0 ) );
		
		glDisableClientState( GL_VERTEX_ARRAY );
		glfwSwapBuffers( main_window );
		glfwPollEvents();
	}
}
catch( std::exception &e )
{
	std::cerr << e.what() << std::endl;
	std::cerr << "Press Enter to Exit" << std::endl;
	while( std::getchar() != '\n' );
}
