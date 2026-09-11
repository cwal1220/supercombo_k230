#include "display.h"

#include <drm/drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

std::string fourcc_name(uint32_t fourcc)
{
    char name[5] = {
        static_cast<char>(fourcc & 0xff),
        static_cast<char>((fourcc >> 8) & 0xff),
        static_cast<char>((fourcc >> 16) & 0xff),
        static_cast<char>((fourcc >> 24) & 0xff),
        0,
    };
    return std::string(name);
}

bool plane_supports(const display_plane *plane, uint32_t fourcc)
{
    if (!plane || !plane->plane) return false;
    for (uint32_t i = 0; i < plane->plane->count_formats; ++i) {
        if (plane->plane->formats[i] == fourcc)
            return true;
    }
    return false;
}

bool drm_plane_supports(const drmModePlane *plane, uint32_t fourcc)
{
    if (!plane) return false;
    for (uint32_t i = 0; i < plane->count_formats; ++i) {
        if (plane->formats[i] == fourcc)
            return true;
    }
    return false;
}

void dump_drm_planes()
{
    const int fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        std::perror("open /dev/dri/card0");
        std::exit(1);
    }
    drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

    drmModePlaneRes *planes = drmModeGetPlaneResources(fd);
    if (!planes) {
        std::perror("drmModeGetPlaneResources");
        close(fd);
        std::exit(1);
    }

    int argb_count = 0;
    std::fprintf(stderr, "drm plane count=%u\n", planes->count_planes);
    for (uint32_t i = 0; i < planes->count_planes; ++i) {
        drmModePlane *plane = drmModeGetPlane(fd, planes->planes[i]);
        if (!plane) continue;
        if (drm_plane_supports(plane, DRM_FORMAT_ARGB8888))
            ++argb_count;

        std::fprintf(stderr, "plane id=%u possible_crtcs=0x%x formats=",
                     plane->plane_id, plane->possible_crtcs);
        for (uint32_t f = 0; f < plane->count_formats; ++f) {
            if (f) std::fprintf(stderr, ",");
            std::fprintf(stderr, "%s", fourcc_name(plane->formats[f]).c_str());
        }

        drmModeObjectProperties *props =
            drmModeObjectGetProperties(fd, plane->plane_id, DRM_MODE_OBJECT_PLANE);
        std::fprintf(stderr, " props=");
        if (props) {
            for (uint32_t p = 0; p < props->count_props; ++p) {
                drmModePropertyRes *prop = drmModeGetProperty(fd, props->props[p]);
                if (!prop) continue;
                if (p) std::fprintf(stderr, ",");
                std::fprintf(stderr, "%s", prop->name);
                drmModeFreeProperty(prop);
            }
            drmModeFreeObjectProperties(props);
        }
        std::fprintf(stderr, "\n");
        drmModeFreePlane(plane);
    }
    std::fprintf(stderr, "ARGB8888 capable DRM planes=%d\n", argb_count);
    drmModeFreePlaneResources(planes);
    close(fd);
}

void dump_plane(const display_plane *plane)
{
    if (!plane || !plane->plane) return;
    std::fprintf(stderr, "plane id=%u cached_fourcc=%s formats=", plane->plane_id,
                 fourcc_name(plane->fourcc).c_str());
    for (uint32_t i = 0; i < plane->plane->count_formats; ++i) {
        if (i) std::fprintf(stderr, ",");
        std::fprintf(stderr, "%s", fourcc_name(plane->plane->formats[i]).c_str());
    }
    std::fprintf(stderr, " props=");
    for (uint8_t i = 0; i < plane->props_count; ++i) {
        if (!plane->props[i]) continue;
        if (i) std::fprintf(stderr, ",");
        std::fprintf(stderr, "%s", plane->props[i]->name);
    }
    std::fprintf(stderr, "\n");
}

void fill_rect(display_buffer *buffer, uint8_t b, uint8_t g, uint8_t r, uint8_t a,
               unsigned x0, unsigned y0, unsigned w, unsigned h)
{
    auto *dst = static_cast<uint8_t *>(buffer->map);
    const unsigned x1 = std::min(buffer->width, x0 + w);
    const unsigned y1 = std::min(buffer->height, y0 + h);
    for (unsigned y = y0; y < y1; ++y) {
        uint8_t *row = dst + static_cast<size_t>(y) * buffer->stride;
        for (unsigned x = x0; x < x1; ++x) {
            uint8_t *p = row + x * 4;
            p[0] = b;
            p[1] = g;
            p[2] = r;
            p[3] = a;
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    const bool do_commit = argc > 1 && std::strcmp(argv[1], "--commit") == 0;
    const bool use_display_get = argc > 1 && std::strcmp(argv[1], "--display-get") == 0;

    if (!do_commit && !use_display_get) {
        dump_drm_planes();
        return 0;
    }

    display *d = display_init(0);
    if (!d) {
        std::fprintf(stderr, "display_init failed\n");
        return 1;
    }

    std::fprintf(stderr, "display %ux%u crtc=%u conn=%u\n", d->width, d->height,
                 d->crtc_id, d->conn_id);

    int argb_count = 0;
    int plane_limit = 0;
    for (display_plane *p = d->planes; p && plane_limit++ < 16; p = p->next) {
        dump_plane(p);
        if (plane_supports(p, DRM_FORMAT_ARGB8888))
            ++argb_count;
    }
    std::fprintf(stderr, "ARGB8888 capable display planes=%d\n", argb_count);

    display_plane *p1 = display_get_plane(d, DRM_FORMAT_ARGB8888);
    display_plane *p2 = display_get_plane(d, DRM_FORMAT_ARGB8888);
    std::fprintf(stderr, "display_get_plane ARGB #1=%p id=%u #2=%p id=%u same=%d\n",
                 static_cast<void *>(p1), p1 ? p1->plane_id : 0,
                 static_cast<void *>(p2), p2 ? p2->plane_id : 0,
                 p1 && p2 && p1 == p2 ? 1 : 0);

    if (do_commit && p1 && p2 && p1 != p2) {
        display_buffer *b1 = display_allocate_buffer(p1, d->width, d->height);
        display_buffer *b2 = display_allocate_buffer(p2, d->width, d->height);
        if (!b1 || !b2) {
            std::fprintf(stderr, "buffer allocation failed b1=%p b2=%p\n",
                         static_cast<void *>(b1), static_cast<void *>(b2));
        } else {
            std::memset(b1->map, 0, b1->size);
            std::memset(b2->map, 0, b2->size);
            fill_rect(b1, 0, 255, 0, 180, 0, 0, d->width, 90);
            fill_rect(b2, 255, 0, 0, 180, d->width / 4, d->height / 4,
                      d->width / 2, d->height / 2);
            const int rc1 = display_commit_buffer(b1, 0, 0);
            const int rc2 = display_commit_buffer(b2, 0, 0);
            std::printf("commit rc1=%d rc2=%d\n", rc1, rc2);
            std::fflush(stdout);
            sleep(5);
        }
        if (b1) display_free_buffer(b1);
        if (b2) display_free_buffer(b2);
    }

    if (p1) display_free_plane(p1);
    if (p2 && p2 != p1) display_free_plane(p2);
    display_exit(d);
    return 0;
}
