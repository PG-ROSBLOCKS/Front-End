import { Component } from '@angular/core';

@Component({
  selector: 'app-rosblocks-help',
  templateUrl: './rosblocks-help.component.html',
  styleUrls: ['./rosblocks-help.component.css']
})
export class RosblocksHelpComponent {
  dropdowns: { [key: number]: boolean } = {};
  selectedVideo: string | null = null;

  onSearch(event: any): void {
    console.log('Buscando:', event.target.value);
  }

  toggleDropdown(id: number): void {
    this.dropdowns[id] = !this.dropdowns[id];
  }

  selectVideo(videoName: string): void {
    switch(videoName) {
      case 'video1':
        this.selectedVideo = 'https://www.youtube.com/embed/dQw4w9WgXcQ';
        break;
      case 'video2':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'video3':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoA':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoB':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoC':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoX':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoY':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      case 'videoZ':
        this.selectedVideo = 'https://www.youtube.com/embed/eudOwe3Rz-0?si=uLdWZQYNo_lg8UF_';
        break;
      default:
        this.selectedVideo = null;
    }
  }
}
